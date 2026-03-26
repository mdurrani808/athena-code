// Copyright (c) 2025 UMD Loop
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "msgs/action/navigate_to_target.hpp"
#include "msgs/msg/active_target.hpp"
#include "msgs/msg/nav_status.hpp"
#include "msgs/msg/planner_event.hpp"
#include "msgs/srv/lat_lon_to_enu.hpp"
#include "msgs/srv/set_target.hpp"

using namespace std::chrono_literals;

// ── State ──────────────────────────────────────────────────────────────────

enum class State
{
  IDLE,
  NAVIGATING,
  ARRIVING,
  STOPPED_AT_TARGET,
  ABORTING,
  RETURNING,
  STOPPED_AT_RETURN,
  TELEOP
};

static std::string stateToStr(State s)
{
  switch (s) {
    case State::IDLE:              return "IDLE";
    case State::NAVIGATING:        return "NAVIGATING";
    case State::ARRIVING:          return "ARRIVING";
    case State::STOPPED_AT_TARGET: return "STOPPED_AT_TARGET";
    case State::ABORTING:          return "ABORTING";
    case State::RETURNING:         return "RETURNING";
    case State::STOPPED_AT_RETURN: return "STOPPED_AT_RETURN";
    case State::TELEOP:            return "TELEOP";
    default:                       return "UNKNOWN";
  }
}

// ── Node ───────────────────────────────────────────────────────────────────

class MissionExecutive : public rclcpp::Node
{
public:
  using NavAction  = msgs::action::NavigateToTarget;
  using GoalHandle = rclcpp_action::ServerGoalHandle<NavAction>;

  // Target entry stored in the registry and as the active target
  struct TargetEntry
  {
    std::string id;
    double      x_m{0.0};
    double      y_m{0.0};
    uint8_t     target_type{0};
    uint8_t     goal_source{0};   // GPS=0, METER=1
    double      tolerance_m{3.0};
    bool        visited{false};
    geometry_msgs::msg::PoseStamped goal_enu;
  };

  explicit MissionExecutive(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("mission_executive", options)
  {
    declare_parameter("velocity_zero_threshold", 0.05);
    declare_parameter("arrival_hold_time",        1.0);
    declare_parameter("replan_distance_m",         3.0);
    declare_parameter("latlon_to_enu_service",
      std::string("/gps_pose_publisher/latlon_to_enu"));
    // §8: launch file should remap to /odom/ground_truth when using ZED ground-truth odometry
    declare_parameter("odom_topic", std::string("/odom"));

    velocity_zero_threshold_ = get_parameter("velocity_zero_threshold").as_double();
    arrival_hold_time_       = get_parameter("arrival_hold_time").as_double();
    replan_distance_m_       = get_parameter("replan_distance_m").as_double();
    const auto latlon_svc    = get_parameter("latlon_to_enu_service").as_string();
    const auto odom_topic    = get_parameter("odom_topic").as_string();

    // Reentrant group — allows the action server / service client to block
    // inside handleAccepted() while other callbacks still run on other threads.
    reentrant_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // ── Publishers ──────────────────────────────────────────────────────────
    goal_pub_          = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    nav_enabled_pub_   = create_publisher<std_msgs::msg::Bool>(
      "/nav_enabled", rclcpp::QoS(1).reliable());
    nav_mode_pub_      = create_publisher<std_msgs::msg::String>("/nav_mode", 10);
    active_target_pub_ = create_publisher<msgs::msg::ActiveTarget>("/active_target", 10);
    nav_status_pub_    = create_publisher<msgs::msg::NavStatus>(
      "/nav_status", rclcpp::QoS(1).reliable());

    // ── Service client ──────────────────────────────────────────────────────
    latlon_client_ = create_client<msgs::srv::LatLonToENU>(
      latlon_svc,
      rmw_qos_profile_services_default,
      reentrant_group_);

    // ── Action server ───────────────────────────────────────────────────────
    action_server_ = rclcpp_action::create_server<NavAction>(
      this,
      "~/navigate_to_target",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const NavAction::Goal> goal) {
        return handleGoal(goal);
      },
      [this](std::shared_ptr<GoalHandle> gh) {
        return handleCancel(gh);
      },
      [this](std::shared_ptr<GoalHandle> gh) {
        handleAccepted(gh);
      },
      rcl_action_server_get_default_options(),
      reentrant_group_);

    // ── Services ────────────────────────────────────────────────────────────
    abort_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/abort",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr res)
      {
        std::lock_guard<std::mutex> lk(mutex_);
        if (state_ == State::IDLE || state_ == State::TELEOP ||
            state_ == State::ABORTING ||
            state_ == State::STOPPED_AT_TARGET || state_ == State::STOPPED_AT_RETURN)
        {
          res->success = false;
          res->message = "Nothing to abort in state " + stateToStr(state_);
          return;
        }
        transition(State::ABORTING, "~/abort service called");
        res->success = true;
        res->message = "Aborting";
      });

    set_target_srv_ = create_service<msgs::srv::SetTarget>(
      "~/set_target",
      [this](
        const msgs::srv::SetTarget::Request::SharedPtr req,
        msgs::srv::SetTarget::Response::SharedPtr res)
      {
        onSetTarget(req, res);
      },
      rmw_qos_profile_services_default,
      reentrant_group_);

    teleop_srv_ = create_service<std_srvs::srv::SetBool>(
      "~/teleop",
      [this](
        const std_srvs::srv::SetBool::Request::SharedPtr req,
        std_srvs::srv::SetBool::Response::SharedPtr res)
      {
        std::lock_guard<std::mutex> lk(mutex_);
        if (req->data) {
          // TELEOP_ON — valid from any state
          transition(State::TELEOP, "teleop enabled");
          res->success = true;
          res->message = "Teleop ON";
        } else {
          // TELEOP_OFF — only from TELEOP
          if (state_ == State::TELEOP) {
            transition(State::IDLE, "teleop disabled");
            res->success = true;
            res->message = "Teleop OFF";
          } else {
            res->success = false;
            res->message = "Not in TELEOP state";
          }
        }
      });

    // ── Subscriptions ────────────────────────────────────────────────────────
    robot_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/robot_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        robot_pose_ = *msg;
        // Arrival check first: once we transition to ARRIVING,
        // checkCrossTrackError's state guard prevents a spurious replan.
        checkArrival();
        checkCrossTrackError();
      });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        const auto & lin = msg->twist.twist.linear;
        robot_speed_ = std::hypot(lin.x, lin.y);
        checkStopDetection();
      });

    planner_event_sub_ = create_subscription<msgs::msg::PlannerEvent>(
      "/planner_event", 10,
      [this](const msgs::msg::PlannerEvent::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        last_planner_event_ = msg->event;
        if (msg->event == msgs::msg::PlannerEvent::PLAN_FAILED) {
          onPlanFailed();
        }
      });

    auto transient_qos = rclcpp::QoS(1).transient_local();
    global_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/global_path", transient_qos,
      [this](const nav_msgs::msg::Path::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        global_path_ = *msg;
      });

    // ── 2 Hz status timer ────────────────────────────────────────────────────
    status_timer_ = create_wall_timer(
      500ms,
      [this]() {
        std::lock_guard<std::mutex> lk(mutex_);
        publishStatus();
        checkActionResult();
      });

    // Publish initial safe state
    publishNavEnabled(false);
    publishNavMode();

    RCLCPP_INFO(get_logger(), "MissionExecutive ready — state: IDLE");
  }

private:
  // ── State transitions ─────────────────────────────────────────────────────

  // Must be called with mutex_ held.
  void transition(State new_state, const std::string & reason)
  {
    RCLCPP_INFO(get_logger(), "[mission_executive] %s → %s: %s",
      stateToStr(state_).c_str(), stateToStr(new_state).c_str(), reason.c_str());
    state_ = new_state;

    const bool nav_active =
      state_ == State::NAVIGATING ||
      state_ == State::ARRIVING   ||
      state_ == State::RETURNING;
    publishNavEnabled(nav_active);
    publishNavMode();
    publishStatus();

    if (state_ == State::TELEOP || !nav_active) {
      // Clear stop-detection tracking on any non-navigating transition
      low_speed_tracking_ = false;
    }
  }

  // ── Internal publish helpers (no lock needed — caller holds mutex) ─────────

  void publishNavEnabled(bool enabled)
  {
    std_msgs::msg::Bool msg;
    msg.data = enabled;
    nav_enabled_pub_->publish(msg);
  }

  void publishNavMode()
  {
    std_msgs::msg::String msg;
    switch (state_) {
      case State::TELEOP:
        msg.data = "teleop"; break;
      case State::NAVIGATING:
      case State::ARRIVING:
      case State::RETURNING:
        msg.data = "autonomous"; break;
      default:
        msg.data = "stopped"; break;
    }
    nav_mode_pub_->publish(msg);
  }

  void publishActiveTarget()
  {
    if (!active_target_.has_value()) return;
    msgs::msg::ActiveTarget at;
    at.target_id   = active_target_->id;
    at.target_type = active_target_->target_type;
    at.tolerance_m = active_target_->tolerance_m;
    at.goal_enu    = active_target_->goal_enu;
    at.goal_source = active_target_->goal_source;
    at.status      = stateToStr(state_);
    active_target_pub_->publish(at);
  }

  void publishStatus()
  {
    msgs::msg::NavStatus s;
    s.state = stateToStr(state_);
    if (active_target_.has_value()) {
      s.active_target_id   = active_target_->id;
      s.active_target_type = active_target_->target_type;
      s.goal_source        = active_target_->goal_source;
    }
    s.distance_to_goal_m  = distToGoal();
    s.cross_track_error_m = computeCrossTrackError();
    s.heading_error_rad   = computeHeadingError();
    s.robot_speed_mps     = robot_speed_;
    s.is_return           = is_return_;
    s.last_planner_event  = last_planner_event_;
    nav_status_pub_->publish(s);
  }

  // ── Geometry helpers ──────────────────────────────────────────────────────

  // Both return -1.0 if data is unavailable.  Called with mutex_ held.

  static double quaternionToYaw(const geometry_msgs::msg::Quaternion & q)
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  double computeHeadingError() const
  {
    if (!robot_pose_.has_value() || !active_target_.has_value()) return 0.0;
    const double yaw = quaternionToYaw(robot_pose_->pose.orientation);
    const double dx = active_target_->goal_enu.pose.position.x - robot_pose_->pose.position.x;
    const double dy = active_target_->goal_enu.pose.position.y - robot_pose_->pose.position.y;
    const double bearing = std::atan2(dy, dx);
    double err = bearing - yaw;
    while (err >  M_PI) err -= 2.0 * M_PI;
    while (err < -M_PI) err += 2.0 * M_PI;
    return err;
  }

  double distToGoal() const
  {
    if (!robot_pose_.has_value() || !active_target_.has_value()) return -1.0;
    const double dx =
      robot_pose_->pose.position.x - active_target_->goal_enu.pose.position.x;
    const double dy =
      robot_pose_->pose.position.y - active_target_->goal_enu.pose.position.y;
    return std::hypot(dx, dy);
  }

  double computeCrossTrackError() const
  {
    if (!robot_pose_.has_value() || !global_path_.has_value()) return -1.0;
    const auto & poses = global_path_->poses;
    if (poses.size() < 2) return -1.0;

    const double rx = robot_pose_->pose.position.x;
    const double ry = robot_pose_->pose.position.y;
    double min_dist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < poses.size() - 1; ++i) {
      const double ax = poses[i].pose.position.x,  ay = poses[i].pose.position.y;
      const double bx = poses[i+1].pose.position.x, by = poses[i+1].pose.position.y;
      const double dx = bx - ax, dy = by - ay;
      const double len2 = dx*dx + dy*dy;
      double dist;
      if (len2 < 1e-10) {
        dist = std::hypot(rx - ax, ry - ay);
      } else {
        const double t = std::clamp(((rx-ax)*dx + (ry-ay)*dy) / len2, 0.0, 1.0);
        dist = std::hypot(rx - (ax + t*dx), ry - (ay + t*dy));
      }
      min_dist = std::min(min_dist, dist);
    }
    return min_dist;
  }

  // ── Subscription callbacks (mutex_ already held by caller) ────────────────

  void checkCrossTrackError()
  {
    if (state_ != State::NAVIGATING && state_ != State::RETURNING) return;
    if (!active_target_.has_value()) return;

    const double xte = computeCrossTrackError();
    if (xte < 0.0 || xte <= replan_distance_m_) return;

    RCLCPP_INFO(get_logger(),
      "[mission_executive] replan triggered: xtrack=%.2fm", xte);
    goal_pub_->publish(active_target_->goal_enu);
  }

  void checkArrival()
  {
    // Transition NAVIGATING/RETURNING → ARRIVING when within tolerance
    if (state_ != State::NAVIGATING && state_ != State::RETURNING) return;
    if (!active_target_.has_value()) return;

    const double d = distToGoal();
    if (d < 0.0 || d >= active_target_->tolerance_m) return;

    transition(State::ARRIVING,
      "within tolerance (d=" + std::to_string(d) + "m)");
  }

  void checkStopDetection()
  {
    // Transition ARRIVING → STOPPED_AT_TARGET / STOPPED_AT_RETURN
    if (state_ != State::ARRIVING) return;

    if (robot_speed_ < velocity_zero_threshold_) {
      if (!low_speed_tracking_) {
        low_speed_start_    = now();
        low_speed_tracking_ = true;
      } else {
        const double held = (now() - low_speed_start_).seconds();
        if (held >= arrival_hold_time_) {
          low_speed_tracking_ = false;
          if (is_return_) {
            transition(State::STOPPED_AT_RETURN, "velocity held < threshold");
          } else {
            // Mark target as visited so RETURN is permitted later
            if (active_target_.has_value()) {
              auto it = target_registry_.find(active_target_->id);
              if (it != target_registry_.end()) {
                it->second.visited = true;
              }
            }
            transition(State::STOPPED_AT_TARGET, "velocity held < threshold");
          }
        }
      }
    } else {
      low_speed_tracking_ = false;
    }
  }

  void onPlanFailed()
  {
    // Called with mutex_ held
    if (state_ == State::NAVIGATING || state_ == State::RETURNING) {
      RCLCPP_WARN(get_logger(),
        "[mission_executive] PLAN_FAILED — transitioning to ABORTING");
      transition(State::ABORTING, "PLAN_FAILED received");
    }
  }

  // ── Action result dispatch (called from the 2 Hz status timer) ────────────
  // Must be called with mutex_ held.

  void checkActionResult()
  {
    if (!active_goal_handle_) return;

    auto result = std::make_shared<NavAction::Result>();

    // Cancel requested externally — result already sent, go straight to IDLE.
    // (ABORTING is only for the case where the result hasn't been sent yet.)
    if (active_goal_handle_->is_canceling()) {
      result->success = false;
      result->message = "Cancelled";
      active_goal_handle_->canceled(result);
      active_goal_handle_ = nullptr;
      transition(State::IDLE, "action cancelled");
      return;
    }

    switch (state_) {
      case State::STOPPED_AT_TARGET:
      case State::STOPPED_AT_RETURN:
        result->success = true;
        result->message = "Arrived at target";
        active_goal_handle_->succeed(result);
        active_goal_handle_ = nullptr;
        return;

      case State::ABORTING:
        result->success = false;
        result->message = "Aborted";
        active_goal_handle_->abort(result);
        active_goal_handle_ = nullptr;
        if (queued_goal_handle_) {
          // Promote queued goal — go directly to NAVIGATING/RETURNING (§3.2.2)
          active_target_      = queued_entry_;
          is_return_          = queued_is_return_;
          active_goal_handle_ = queued_goal_handle_;
          queued_goal_handle_ = nullptr;
          transition(is_return_ ? State::RETURNING : State::NAVIGATING,
            "queued goal dequeued after abort");
          goal_pub_->publish(active_target_->goal_enu);
          publishActiveTarget();
        } else {
          transition(State::IDLE, "abort complete");
        }
        return;

      default:
        break;
    }

    // Still executing — push feedback
    auto fb = std::make_shared<NavAction::Feedback>();
    fb->distance_to_goal_m  = distToGoal();
    fb->cross_track_error_m = computeCrossTrackError();
    fb->state               = stateToStr(state_);
    active_goal_handle_->publish_feedback(fb);
  }

  // ── Action server callbacks ───────────────────────────────────────────────

  rclcpp_action::GoalResponse handleGoal(
    std::shared_ptr<const NavAction::Goal> /*goal*/)
  {
    // Accept all goals; teleop/state guards are applied in handleAccepted.
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(std::shared_ptr<GoalHandle> /*gh*/)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // Runs on a thread from the MultiThreadedExecutor (reentrant group).
  // May block briefly for the latlon_to_enu service call.
  void handleAccepted(std::shared_ptr<GoalHandle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();

    // ── Resolve ENU coordinates ────────────────────────────────────────────
    TargetEntry entry;
    const bool is_ret = goal->is_return;

    if (!goal->target_id.empty()) {
      // Look up pre-registered target
      std::lock_guard<std::mutex> lk(mutex_);
      auto it = target_registry_.find(goal->target_id);
      if (it == target_registry_.end()) {
        auto res = std::make_shared<NavAction::Result>();
        res->success = false;
        res->message = "Unknown target_id: " + goal->target_id;
        goal_handle->abort(res);
        return;
      }
      if (is_ret && !it->second.visited) {
        auto res = std::make_shared<NavAction::Result>();
        res->success = false;
        res->message = "Target not yet visited — cannot RETURN";
        goal_handle->abort(res);
        return;
      }
      entry = it->second;
    } else {
      // Inline goal — no target_id
      entry.id          = "";
      entry.target_type = goal->target_type;
      entry.tolerance_m = goal->tolerance_m;
      entry.goal_source = goal->goal_type;

      if (goal->goal_type == NavAction::Goal::GPS) {
        if (!latlon_client_->wait_for_service(3s)) {
          auto res = std::make_shared<NavAction::Result>();
          res->success = false;
          res->message = "latlon_to_enu service not available";
          goal_handle->abort(res);
          return;
        }
        auto req = std::make_shared<msgs::srv::LatLonToENU::Request>();
        req->lat = goal->lat;
        req->lon = goal->lon;
        auto future = latlon_client_->async_send_request(req);
        // Blocking wait is safe here: MultiThreadedExecutor + reentrant group
        // means other callbacks continue on other threads.
        if (future.wait_for(5s) != std::future_status::ready) {
          auto res = std::make_shared<NavAction::Result>();
          res->success = false;
          res->message = "latlon_to_enu service timeout";
          goal_handle->abort(res);
          return;
        }
        auto resp = future.get();
        entry.x_m = resp->x;
        entry.y_m = resp->y;
      } else {
        entry.x_m = goal->x_m;
        entry.y_m = goal->y_m;
      }

      entry.goal_enu.header.frame_id = "map";
      entry.goal_enu.header.stamp    = now();
      entry.goal_enu.pose.position.x = entry.x_m;
      entry.goal_enu.pose.position.y = entry.y_m;
      entry.goal_enu.pose.orientation.w = 1.0;
    }

    // ── Apply state transition ─────────────────────────────────────────────
    {
      std::lock_guard<std::mutex> lk(mutex_);

      if (state_ == State::TELEOP) {
        auto res = std::make_shared<NavAction::Result>();
        res->success = false;
        res->message = "Cannot navigate — currently in TELEOP mode";
        goal_handle->abort(res);
        return;
      }

      // §3.2.2: During ABORTING, queue the goal rather than preempting the abort
      if (state_ == State::ABORTING) {
        // Preempt any previously queued goal
        if (queued_goal_handle_ && queued_goal_handle_->is_active()) {
          auto old_res = std::make_shared<NavAction::Result>();
          old_res->success = false;
          old_res->message = "Preempted by newer queued goal";
          queued_goal_handle_->abort(old_res);
        }
        queued_goal_handle_  = goal_handle;
        queued_entry_        = entry;
        queued_is_return_    = is_ret;
        RCLCPP_INFO(get_logger(),
          "[mission_executive] goal queued — currently ABORTING");
        return;
      }

      // Preempt any existing active goal (guard: abort() throws if not active)
      if (active_goal_handle_ && active_goal_handle_->is_active()) {
        auto old_res = std::make_shared<NavAction::Result>();
        old_res->success = false;
        old_res->message = "Preempted by new goal";
        active_goal_handle_->abort(old_res);
        active_goal_handle_ = nullptr;
      }

      active_target_  = entry;
      is_return_      = is_ret;
      active_goal_handle_ = goal_handle;

      transition(is_ret ? State::RETURNING : State::NAVIGATING,
        is_ret ? "RETURN action accepted" : "GO_TO action accepted");

      // Trigger global planner
      goal_pub_->publish(active_target_->goal_enu);
      publishActiveTarget();
    }
  }

  // ── SetTarget service ─────────────────────────────────────────────────────

  void onSetTarget(
    const msgs::srv::SetTarget::Request::SharedPtr req,
    msgs::srv::SetTarget::Response::SharedPtr res)
  {
    TargetEntry entry;
    entry.id          = req->target_id;
    entry.target_type = req->target_type;
    entry.tolerance_m = req->tolerance_m;
    entry.goal_source = req->goal_type;

    if (req->goal_type == msgs::srv::SetTarget::Request::GPS) {
      if (!latlon_client_->wait_for_service(3s)) {
        res->success = false;
        res->message = "latlon_to_enu service not available";
        return;
      }
      auto conv = std::make_shared<msgs::srv::LatLonToENU::Request>();
      conv->lat = req->lat;
      conv->lon = req->lon;
      auto future = latlon_client_->async_send_request(conv);
      if (future.wait_for(5s) != std::future_status::ready) {
        res->success = false;
        res->message = "latlon_to_enu service timeout";
        return;
      }
      auto resp = future.get();
      entry.x_m = resp->x;
      entry.y_m = resp->y;
    } else {
      entry.x_m = req->x_m;
      entry.y_m = req->y_m;
    }

    entry.goal_enu.header.frame_id    = "map";
    entry.goal_enu.header.stamp       = now();
    entry.goal_enu.pose.position.x    = entry.x_m;
    entry.goal_enu.pose.position.y    = entry.y_m;
    entry.goal_enu.pose.orientation.w = 1.0;

    {
      std::lock_guard<std::mutex> lk(mutex_);
      target_registry_[entry.id] = entry;
    }

    res->success = true;
    res->message = "Target '" + entry.id + "' registered";
    RCLCPP_INFO(get_logger(),
      "Target '%s' registered: (%.2f, %.2f) tol=%.2fm",
      entry.id.c_str(), entry.x_m, entry.y_m, entry.tolerance_m);
  }

  // ── Member data ───────────────────────────────────────────────────────────

  std::mutex mutex_;
  State      state_{State::IDLE};

  std::unordered_map<std::string, TargetEntry> target_registry_;
  std::optional<TargetEntry>                   active_target_;
  bool                                          is_return_{false};

  // Goal queued while ABORTING (§3.2.2 — GO_TO/RETURN during ABORTING are "queued")
  std::shared_ptr<GoalHandle> queued_goal_handle_;
  TargetEntry                 queued_entry_;
  bool                        queued_is_return_{false};

  // Sensor cache
  std::optional<geometry_msgs::msg::PoseStamped> robot_pose_;
  std::optional<nav_msgs::msg::Path>             global_path_;
  double                                          robot_speed_{0.0};
  rclcpp::Time                                    low_speed_start_{0, 0, RCL_ROS_TIME};
  bool                                            low_speed_tracking_{false};
  uint8_t                                         last_planner_event_{0};

  // Parameters
  double velocity_zero_threshold_{0.05};
  double arrival_hold_time_{1.0};
  double replan_distance_m_{3.0};

  // ROS interfaces
  rclcpp::CallbackGroup::SharedPtr reentrant_group_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr             nav_enabled_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           nav_mode_pub_;
  rclcpp::Publisher<msgs::msg::ActiveTarget>::SharedPtr         active_target_pub_;
  rclcpp::Publisher<msgs::msg::NavStatus>::SharedPtr            nav_status_pub_;

  rclcpp_action::Server<NavAction>::SharedPtr action_server_;
  std::shared_ptr<GoalHandle>                 active_goal_handle_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr  abort_srv_;
  rclcpp::Service<msgs::srv::SetTarget>::SharedPtr     set_target_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr   teleop_srv_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr         odom_sub_;
  rclcpp::Subscription<msgs::msg::PlannerEvent>::SharedPtr         planner_event_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr             global_path_sub_;

  rclcpp::Client<msgs::srv::LatLonToENU>::SharedPtr latlon_client_;

  rclcpp::TimerBase::SharedPtr status_timer_;
};

// ── main ──────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MissionExecutive>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
