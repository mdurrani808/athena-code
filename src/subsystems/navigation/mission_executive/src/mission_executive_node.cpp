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
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "vision_msgs/msg/detection2_d.hpp"

#include "msgs/action/navigate_to_target.hpp"
#include "msgs/msg/active_target.hpp"
#include "msgs/msg/nav_status.hpp"
#include "msgs/msg/planner_event.hpp"
#include "msgs/srv/lat_lon_to_enu.hpp"
#include "msgs/srv/set_target.hpp"

using namespace std::chrono_literals;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

// ─── Types ───────────────────────────────────────────────────────────────────

enum class State {
  IDLE,
  NAVIGATING,
  ARRIVING,
  STOPPED_AT_TARGET,
  SPIRAL_COVERAGE,
  SPIRAL_DONE,
  ABORTING,
  RETURNING,
  STOPPED_AT_RETURN,
  TELEOP
};

static std::string stateToStr(State s) {
  switch (s) {
    case State::IDLE:              return "IDLE";
    case State::NAVIGATING:        return "NAVIGATING";
    case State::ARRIVING:          return "ARRIVING";
    case State::STOPPED_AT_TARGET: return "STOPPED_AT_TARGET";
    case State::SPIRAL_COVERAGE:   return "SPIRAL_COVERAGE";
    case State::SPIRAL_DONE:       return "SPIRAL_DONE";
    case State::ABORTING:          return "ABORTING";
    case State::RETURNING:         return "RETURNING";
    case State::STOPPED_AT_RETURN: return "STOPPED_AT_RETURN";
    case State::TELEOP:            return "TELEOP";
    default:                       return "UNKNOWN";
  }
}

struct Pose2D {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct TargetEntry {
  std::string id;
  double x_m{0.0};
  double y_m{0.0};
  uint8_t target_type{0};
  uint8_t goal_source{0};
  double tolerance_m{3.0};
  bool visited{false};
};

struct MissionParams {
  double stop_angular_vel_threshold{0.05};
  double arrival_hold_time{1.0};
  double replan_distance_m{3.0};
  double spiral_timeout_s{120.0};
  double spiral_radius_m{15.0};
  double spiral_spacing_m{2.0};
  double spiral_angular_step{0.5};
  double spiral_waypoint_tolerance_m{2.0};
};

struct CommandResult {
  bool success;
  std::string message;
};

struct StartNavResult {
  bool accepted{false};
  std::string message;
  bool preempted_old{false};
  bool publish_goal{false};
  Pose2D goal_to_publish{};
};

struct TickResult {
  bool publish_goal{false};
  Pose2D goal_to_publish;
  bool action_finished{false};
  bool action_success{false};
  std::string action_message;
  bool start_queued_goal{false};
};

// ─── Node ────────────────────────────────────────────────────────────────────

class MissionExecutive : public rclcpp::Node
{
public:
  using NavAction  = msgs::action::NavigateToTarget;
  using GoalHandle = rclcpp_action::ServerGoalHandle<NavAction>;

  explicit MissionExecutive(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("mission_executive", options)
  {
    declare_parameter("stop_angular_vel_threshold", 0.05);
    declare_parameter("arrival_hold_time",           1.0);
    declare_parameter("replan_distance_m",           3.0);
    declare_parameter("latlon_to_enu_service",
      std::string("/gps_pose_publisher/latlon_to_enu"));
    declare_parameter("spiral_timeout_s",            120.0);
    declare_parameter("spiral_radius_m",             15.0);
    declare_parameter("spiral_spacing_m",            2.0);
    declare_parameter("spiral_angular_step",         0.5);
    declare_parameter("spiral_waypoint_tolerance_m", 2.0);
    declare_parameter("aruco_detection_topic", std::string("/aruco_loc"));
    declare_parameter("yolo_detection_topic",  std::string("/yolo_detection"));
    declare_parameter("imu_topic",           std::string("/imu"));
    declare_parameter("planner_event_topic", std::string("/planner_event"));
    declare_parameter("global_path_topic",   std::string("/global_path"));
    declare_parameter("goal_pose_topic",     std::string("/goal_pose"));
    declare_parameter("nav_enabled_topic",   std::string("/nav_enabled"));
    declare_parameter("nav_mode_topic",      std::string("/nav_mode"));
    declare_parameter("active_target_topic", std::string("/active_target"));
    declare_parameter("nav_status_topic",    std::string("/nav_status"));

    params_.stop_angular_vel_threshold  = get_parameter("stop_angular_vel_threshold").as_double();
    params_.arrival_hold_time           = get_parameter("arrival_hold_time").as_double();
    params_.replan_distance_m           = get_parameter("replan_distance_m").as_double();
    params_.spiral_timeout_s            = get_parameter("spiral_timeout_s").as_double();
    params_.spiral_radius_m             = get_parameter("spiral_radius_m").as_double();
    params_.spiral_spacing_m            = get_parameter("spiral_spacing_m").as_double();
    params_.spiral_angular_step         = get_parameter("spiral_angular_step").as_double();
    params_.spiral_waypoint_tolerance_m = get_parameter("spiral_waypoint_tolerance_m").as_double();

    const auto latlon_svc           = get_parameter("latlon_to_enu_service").as_string();
    const auto aruco_detection_topic = get_parameter("aruco_detection_topic").as_string();
    const auto yolo_detection_topic  = get_parameter("yolo_detection_topic").as_string();
    const auto imu_topic            = get_parameter("imu_topic").as_string();
    const auto planner_event_topic  = get_parameter("planner_event_topic").as_string();
    const auto global_path_topic    = get_parameter("global_path_topic").as_string();
    const auto goal_pose_topic      = get_parameter("goal_pose_topic").as_string();
    const auto nav_enabled_topic    = get_parameter("nav_enabled_topic").as_string();
    const auto nav_mode_topic       = get_parameter("nav_mode_topic").as_string();
    const auto active_target_topic  = get_parameter("active_target_topic").as_string();
    const auto nav_status_topic     = get_parameter("nav_status_topic").as_string();

    reentrant_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    goal_pub_          = create_publisher<geometry_msgs::msg::PoseStamped>(goal_pose_topic, 10);
    nav_enabled_pub_   = create_publisher<std_msgs::msg::Bool>(
      nav_enabled_topic, rclcpp::QoS(1).reliable());
    nav_mode_pub_      = create_publisher<std_msgs::msg::String>(nav_mode_topic, 10);
    active_target_pub_ = create_publisher<msgs::msg::ActiveTarget>(active_target_topic, 10);
    nav_status_pub_    = create_publisher<msgs::msg::NavStatus>(
      nav_status_topic, rclcpp::QoS(1).reliable());

    latlon_client_ = create_client<msgs::srv::LatLonToENU>(
      latlon_svc,
      rmw_qos_profile_services_default,
      reentrant_group_);

    action_server_ = rclcpp_action::create_server<NavAction>(
      this,
      "~/navigate_to_target",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const NavAction::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<GoalHandle>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<GoalHandle> gh) { handleAccepted(gh); },
      rcl_action_server_get_default_options(),
      reentrant_group_);

    abort_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/abort",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr res)
      {
        std::lock_guard<std::mutex> lk(mutex_);
        auto cmd = abort();
        res->success = cmd.success;
        res->message = cmd.message;
        if (cmd.success) { publishNavEnabled(); publishNavMode(); publishStatus(); }
      });

    set_target_srv_ = create_service<msgs::srv::SetTarget>(
      "~/set_target",
      [this](
        const msgs::srv::SetTarget::Request::SharedPtr req,
        msgs::srv::SetTarget::Response::SharedPtr res)
      { onSetTarget(req, res); },
      rmw_qos_profile_services_default,
      reentrant_group_);

    teleop_srv_ = create_service<std_srvs::srv::SetBool>(
      "~/teleop",
      [this](
        const std_srvs::srv::SetBool::Request::SharedPtr req,
        std_srvs::srv::SetBool::Response::SharedPtr res)
      {
        std::lock_guard<std::mutex> lk(mutex_);
        auto cmd = setTeleop(req->data);
        res->success = cmd.success;
        res->message = cmd.message;
        if (cmd.success) { publishNavEnabled(); publishNavMode(); publishStatus(); }
      });

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        const auto & w = msg->angular_velocity;
        onImu(std::sqrt(w.x*w.x + w.y*w.y + w.z*w.z), now().seconds());
      });

    planner_event_sub_ = create_subscription<msgs::msg::PlannerEvent>(
      planner_event_topic, 10,
      [this](const msgs::msg::PlannerEvent::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        last_planner_event_ = msg->event;
        if (msg->event == msgs::msg::PlannerEvent::PLAN_FAILED) {
          onPlanFailed();
          publishNavEnabled();
          publishNavMode();
          publishStatus();
        }
      });

    global_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      global_path_topic, rclcpp::QoS(1).transient_local(),
      [this](const nav_msgs::msg::Path::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        std::vector<Pose2D> path;
        path.reserve(msg->poses.size());
        for (const auto& p : msg->poses) {
          path.push_back({p.pose.position.x, p.pose.position.y,
                          quaternionToYaw(p.pose.orientation)});
        }
        global_path_ = std::move(path);
      });

    aruco_sub_ = create_subscription<vision_msgs::msg::Detection2D>(
      aruco_detection_topic, 10,
      [this](const vision_msgs::msg::Detection2D::SharedPtr) {
        std::lock_guard<std::mutex> lk(mutex_);
        onDetection();
      });

    yolo_sub_ = create_subscription<std_msgs::msg::Bool>(
      yolo_detection_topic, 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        if (msg->data) onDetection();
      });

    status_timer_ = create_wall_timer(
      500ms,
      [this]() {
        std::lock_guard<std::mutex> lk(mutex_);
        refreshPoseFromTF();
        auto res = tick(now().seconds());

        if (res.publish_goal) publishGoal(res.goal_to_publish);
        checkActionResult(res);
        publishNavEnabled();
        publishNavMode();
        publishStatus();
      });

    publishNavEnabled();
    publishNavMode();
    RCLCPP_INFO(get_logger(), "MissionExecutive ready — state: IDLE");
  }

private:
  // ── State machine ─────────────────────────────────────────────────────────

  bool isNavEnabled() const {
    return state_ == State::NAVIGATING || state_ == State::ARRIVING ||
           state_ == State::RETURNING  || state_ == State::SPIRAL_COVERAGE;
  }

  std::string getNavMode() const {
    switch (state_) {
      case State::TELEOP:        return "teleop";
      case State::NAVIGATING:
      case State::ARRIVING:
      case State::RETURNING:
      case State::SPIRAL_COVERAGE: return "autonomous";
      default:                   return "stopped";
    }
  }

  void transition(State new_state) {
    state_ = new_state;
    if (!isNavEnabled()) low_speed_tracking_ = false;
  }

  double getDistToGoal() const {
    if (!robot_pose_.has_value() || !active_target_.has_value()) return -1.0;
    return std::hypot(robot_pose_->x - active_target_->x_m,
                      robot_pose_->y - active_target_->y_m);
  }

  double getHeadingError() const {
    if (!robot_pose_.has_value() || !active_target_.has_value()) return 0.0;
    const double dx = active_target_->x_m - robot_pose_->x;
    const double dy = active_target_->y_m - robot_pose_->y;
    double err = std::atan2(dy, dx) - robot_pose_->yaw;
    while (err >  M_PI) err -= 2.0 * M_PI;
    while (err < -M_PI) err += 2.0 * M_PI;
    return err;
  }

  double getCrossTrackError() const {
    if (!robot_pose_.has_value() || !global_path_.has_value()) return -1.0;
    const auto & poses = global_path_.value();
    if (poses.size() < 2) return -1.0;
    const double rx = robot_pose_->x, ry = robot_pose_->y;
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < poses.size() - 1; ++i) {
      const double ax = poses[i].x,    ay = poses[i].y;
      const double bx = poses[i+1].x,  by = poses[i+1].y;
      const double dx = bx - ax,       dy = by - ay;
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

  void onImu(double angular_vel_mag, double current_time_s) {
    imu_angular_vel_ = angular_vel_mag;
    if (state_ != State::ARRIVING) return;
    if (imu_angular_vel_ < params_.stop_angular_vel_threshold) {
      if (!low_speed_tracking_) {
        low_speed_start_s_  = current_time_s;
        low_speed_tracking_ = true;
      } else if (current_time_s - low_speed_start_s_ >= params_.arrival_hold_time) {
        low_speed_tracking_ = false;
        if (is_return_) {
          transition(State::STOPPED_AT_RETURN);
        } else {
          if (active_target_.has_value()) {
            auto it = target_registry_.find(active_target_->id);
            if (it != target_registry_.end()) it->second.visited = true;
          }
          transition(State::STOPPED_AT_TARGET);
        }
      }
    } else {
      low_speed_tracking_ = false;
    }
  }

  void onPlanFailed() {
    if (state_ == State::NAVIGATING || state_ == State::RETURNING)
      transition(State::ABORTING);
  }

  void onDetection() {
    if (state_ == State::SPIRAL_COVERAGE) {
      detection_received_ = true;
      transition(State::SPIRAL_DONE);
    }
  }

  CommandResult setTarget(const TargetEntry& entry) {
    target_registry_[entry.id] = entry;
    return {true, "Target '" + entry.id + "' registered"};
  }

  StartNavResult startNav(const std::string& target_id,
                          const std::optional<TargetEntry>& inline_target,
                          bool is_return) {
    StartNavResult res;
    TargetEntry entry;
    if (!target_id.empty()) {
      auto it = target_registry_.find(target_id);
      if (it == target_registry_.end()) {
        return {false, "Unknown target_id: " + target_id};
      }
      if (is_return && !it->second.visited) {
        return {false, "Target not yet visited — cannot RETURN"};
      }
      entry = it->second;
    } else if (inline_target.has_value()) {
      entry = inline_target.value();
    } else {
      return {false, "No target provided"};
    }

    if (state_ == State::TELEOP)
      return {false, "Cannot navigate — currently in TELEOP mode"};

    if (state_ == State::ABORTING) {
      if (queued_active_) res.preempted_old = true;
      queued_active_    = true;
      queued_entry_     = entry;
      queued_is_return_ = is_return;
      return {true, "Goal queued — currently ABORTING"};
    }

    const bool is_active_nav = (state_ != State::IDLE &&
                                state_ != State::STOPPED_AT_TARGET &&
                                state_ != State::STOPPED_AT_RETURN &&
                                state_ != State::SPIRAL_DONE);
    if (is_active_nav) res.preempted_old = true;

    active_target_ = entry;
    is_return_     = is_return;
    transition(is_return ? State::RETURNING : State::NAVIGATING);

    res.accepted       = true;
    res.publish_goal   = true;
    res.goal_to_publish = {entry.x_m, entry.y_m, 0.0};
    return res;
  }

  CommandResult abort() {
    if (state_ == State::IDLE || state_ == State::TELEOP ||
        state_ == State::ABORTING ||
        state_ == State::STOPPED_AT_TARGET || state_ == State::STOPPED_AT_RETURN ||
        state_ == State::SPIRAL_DONE) {
      return {false, "Nothing to abort in state " + stateToStr(state_)};
    }
    transition(State::ABORTING);
    return {true, "Aborting"};
  }

  CommandResult setTeleop(bool enable) {
    if (enable) {
      transition(State::TELEOP);
      return {true, "Teleop ON"};
    }
    if (state_ != State::TELEOP) return {false, "Not in TELEOP state"};
    transition(State::IDLE);
    return {true, "Teleop OFF"};
  }

  void cancelNav() { transition(State::IDLE); }

  std::vector<Pose2D> generateSpiralWaypoints(const Pose2D& start) {
    std::vector<Pose2D> waypoints;
    const double a = params_.spiral_spacing_m / (2.0 * M_PI);
    if (a <= 0.0) return waypoints;
    const double max_angle   = params_.spiral_radius_m / a;
    const double min_start_r = 1.5;
    const double yaw0        = start.yaw;
    for (double theta = 0.0; theta <= max_angle; theta += params_.spiral_angular_step) {
      const double r = min_start_r + a * theta;
      if (r > params_.spiral_radius_m) break;
      waypoints.push_back({
        start.x + r * std::cos(yaw0 + theta + M_PI_2),
        start.y + r * std::sin(yaw0 + theta + M_PI_2),
        0.0
      });
    }
    return waypoints;
  }

  void startSpiralCoverage(double current_time_s) {
    if (!robot_pose_.has_value()) { transition(State::SPIRAL_DONE); return; }
    spiral_waypoints_    = generateSpiralWaypoints(robot_pose_.value());
    spiral_waypoint_idx_ = 0;
    spiral_start_time_s_ = current_time_s;
    detection_received_  = false;
    if (spiral_waypoints_.empty()) transition(State::SPIRAL_DONE);
  }

  TickResult tick(double current_time_s) {
    TickResult res;

    // 1. Check arrival
    if ((state_ == State::NAVIGATING || state_ == State::RETURNING) &&
        active_target_.has_value()) {
      const double d = getDistToGoal();
      if (d >= 0.0 && d < active_target_->tolerance_m) transition(State::ARRIVING);
    }

    // 2. Check cross-track error → replan
    if ((state_ == State::NAVIGATING || state_ == State::RETURNING) &&
        active_target_.has_value()) {
      const double xte = getCrossTrackError();
      if (xte >= 0.0 && xte > params_.replan_distance_m) {
        res.publish_goal    = true;
        res.goal_to_publish = {active_target_->x_m, active_target_->y_m, 0.0};
      }
    }

    // 3. Check spiral progress
    if (state_ == State::SPIRAL_COVERAGE) {
      if (current_time_s - spiral_start_time_s_ >= params_.spiral_timeout_s) {
        transition(State::SPIRAL_DONE);
      } else if (robot_pose_.has_value() && !spiral_waypoints_.empty()) {
        const auto & wp = spiral_waypoints_[spiral_waypoint_idx_];
        if (std::hypot(robot_pose_->x - wp.x, robot_pose_->y - wp.y)
            < params_.spiral_waypoint_tolerance_m) {
          ++spiral_waypoint_idx_;
          if (spiral_waypoint_idx_ >= spiral_waypoints_.size()) {
            transition(State::SPIRAL_DONE);
          } else {
            res.publish_goal    = true;
            res.goal_to_publish = spiral_waypoints_[spiral_waypoint_idx_];
          }
        }
      }
    }

    // 4. Action result resolution
    if (state_ == State::STOPPED_AT_TARGET) {
      const bool needs_spiral = active_target_.has_value() &&
        (active_target_->target_type == 1 || active_target_->target_type == 2);
      if (needs_spiral) {
        transition(State::SPIRAL_COVERAGE);
        startSpiralCoverage(current_time_s);
        if (state_ == State::SPIRAL_COVERAGE && !spiral_waypoints_.empty()) {
          res.publish_goal    = true;
          res.goal_to_publish = spiral_waypoints_[0];
        }
      } else {
        res.action_finished = true;
        res.action_success  = true;
        res.action_message  = "Arrived at target";
      }
    } else if (state_ == State::SPIRAL_DONE) {
      res.action_finished = true;
      res.action_success  = true;
      res.action_message  = detection_received_
        ? "Detection found during spiral coverage"
        : "Spiral coverage complete (timeout or all waypoints visited)";
    } else if (state_ == State::STOPPED_AT_RETURN) {
      res.action_finished = true;
      res.action_success  = true;
      res.action_message  = "Arrived at target";
    } else if (state_ == State::ABORTING) {
      res.action_finished = true;
      res.action_success  = false;
      res.action_message  = "Aborted";
      if (queued_active_) {
        active_target_  = queued_entry_;
        is_return_      = queued_is_return_;
        queued_active_  = false;
        transition(is_return_ ? State::RETURNING : State::NAVIGATING);
        res.start_queued_goal = true;
        res.publish_goal      = true;
        res.goal_to_publish   = {active_target_->x_m, active_target_->y_m, 0.0};
      } else {
        transition(State::IDLE);
      }
    }

    return res;
  }

  // ── ROS helpers ───────────────────────────────────────────────────────────

  static double quaternionToYaw(const geometry_msgs::msg::Quaternion & q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  void refreshPoseFromTF() {
    try {
      const auto tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
      robot_pose_ = Pose2D{
        tf.transform.translation.x,
        tf.transform.translation.y,
        quaternionToYaw(tf.transform.rotation)
      };
    } catch (const tf2::TransformException &) {}
  }

  void publishGoal(const Pose2D& goal) {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id    = "map";
    p.header.stamp       = now();
    p.pose.position.x    = goal.x;
    p.pose.position.y    = goal.y;
    p.pose.orientation.w = 1.0;
    goal_pub_->publish(p);
  }

  void publishNavEnabled() {
    std_msgs::msg::Bool msg;
    msg.data = isNavEnabled();
    nav_enabled_pub_->publish(msg);
  }

  void publishNavMode() {
    std_msgs::msg::String msg;
    msg.data = getNavMode();
    nav_mode_pub_->publish(msg);
  }

  void publishActiveTarget() {
    if (!active_target_.has_value()) return;
    msgs::msg::ActiveTarget at;
    at.target_id   = active_target_->id;
    at.target_type = active_target_->target_type;
    at.tolerance_m = active_target_->tolerance_m;
    at.goal_enu.header.frame_id  = "map";
    at.goal_enu.header.stamp     = now();
    at.goal_enu.pose.position.x  = active_target_->x_m;
    at.goal_enu.pose.position.y  = active_target_->y_m;
    at.goal_enu.pose.orientation.w = 1.0;
    at.goal_source = active_target_->goal_source;
    at.status      = stateToStr(state_);
    active_target_pub_->publish(at);
  }

  void publishStatus() {
    msgs::msg::NavStatus s;
    s.state = stateToStr(state_);
    if (active_target_.has_value()) {
      s.active_target_id   = active_target_->id;
      s.active_target_type = active_target_->target_type;
      s.goal_source        = active_target_->goal_source;
    }
    s.distance_to_goal_m  = getDistToGoal();
    s.cross_track_error_m = getCrossTrackError();
    s.heading_error_rad   = getHeadingError();
    s.robot_speed_mps     = imu_angular_vel_;
    s.is_return           = is_return_;
    s.last_planner_event  = last_planner_event_;
    nav_status_pub_->publish(s);
  }

  void checkActionResult(const TickResult& res) {
    if (!active_goal_handle_) return;

    if (active_goal_handle_->is_canceling()) {
      auto result     = std::make_shared<NavAction::Result>();
      result->success = false;
      result->message = "Cancelled";
      active_goal_handle_->canceled(result);
      active_goal_handle_ = nullptr;
      cancelNav();
      return;
    }

    if (res.action_finished) {
      auto result     = std::make_shared<NavAction::Result>();
      result->success = res.action_success;
      result->message = res.action_message;
      if (res.action_success) active_goal_handle_->succeed(result);
      else                    active_goal_handle_->abort(result);
      active_goal_handle_ = nullptr;
    } else {
      auto fb                   = std::make_shared<NavAction::Feedback>();
      fb->distance_to_goal_m    = getDistToGoal();
      fb->cross_track_error_m   = getCrossTrackError();
      fb->state                 = stateToStr(state_);
      active_goal_handle_->publish_feedback(fb);
    }

    if (res.start_queued_goal) {
      active_goal_handle_ = queued_goal_handle_;
      queued_goal_handle_ = nullptr;
      publishActiveTarget();
    }
  }

  void handleAccepted(std::shared_ptr<GoalHandle> goal_handle) {
    const auto goal = goal_handle->get_goal();

    std::optional<TargetEntry> inline_target;
    if (goal->target_id.empty()) {
      TargetEntry entry;
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
        if (future.wait_for(5s) != std::future_status::ready) {
          auto res = std::make_shared<NavAction::Result>();
          res->success = false;
          res->message = "latlon_to_enu service timeout";
          goal_handle->abort(res);
          return;
        }
        const auto resp = future.get();
        entry.x_m = resp->x;
        entry.y_m = resp->y;
      } else {
        entry.x_m = goal->x_m;
        entry.y_m = goal->y_m;
      }
      inline_target = entry;
    }

    {
      std::lock_guard<std::mutex> lk(mutex_);
      auto cmd = startNav(goal->target_id, inline_target, goal->is_return);

      if (!cmd.accepted) {
        auto res = std::make_shared<NavAction::Result>();
        res->success = false;
        res->message = cmd.message;
        goal_handle->abort(res);
        return;
      }

      if (queued_active_ && state_ == State::ABORTING) {
        if (queued_goal_handle_ && queued_goal_handle_->is_active()) {
          auto old = std::make_shared<NavAction::Result>();
          old->success = false;
          old->message = "Preempted by newer queued goal";
          queued_goal_handle_->abort(old);
        }
        queued_goal_handle_ = goal_handle;
        return;
      }

      if (cmd.preempted_old && active_goal_handle_ && active_goal_handle_->is_active()) {
        auto old = std::make_shared<NavAction::Result>();
        old->success = false;
        old->message = "Preempted by new goal";
        active_goal_handle_->abort(old);
        active_goal_handle_ = nullptr;
      }

      active_goal_handle_ = goal_handle;

      if (cmd.publish_goal) {
        publishGoal(cmd.goal_to_publish);
        publishActiveTarget();
      }
      publishNavEnabled();
      publishNavMode();
      publishStatus();
    }
  }

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
      const auto resp = future.get();
      entry.x_m = resp->x;
      entry.y_m = resp->y;
    } else {
      entry.x_m = req->x_m;
      entry.y_m = req->y_m;
    }

    std::lock_guard<std::mutex> lk(mutex_);
    auto cmd     = setTarget(entry);
    res->success = cmd.success;
    res->message = cmd.message;
  }

  // ── State ─────────────────────────────────────────────────────────────────

  std::mutex mutex_;
  MissionParams params_;
  State state_{State::IDLE};

  std::unordered_map<std::string, TargetEntry> target_registry_;
  std::optional<TargetEntry> active_target_;
  bool is_return_{false};

  bool queued_active_{false};
  TargetEntry queued_entry_;
  bool queued_is_return_{false};

  std::optional<Pose2D> robot_pose_;
  std::optional<std::vector<Pose2D>> global_path_;

  double imu_angular_vel_{0.0};
  double low_speed_start_s_{0.0};
  bool low_speed_tracking_{false};
  uint8_t last_planner_event_{0};

  std::vector<Pose2D> spiral_waypoints_;
  size_t spiral_waypoint_idx_{0};
  double spiral_start_time_s_{0.0};
  bool detection_received_{false};

  // ── ROS handles ───────────────────────────────────────────────────────────

  rclcpp::CallbackGroup::SharedPtr reentrant_group_;

  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr             nav_enabled_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           nav_mode_pub_;
  rclcpp::Publisher<msgs::msg::ActiveTarget>::SharedPtr         active_target_pub_;
  rclcpp::Publisher<msgs::msg::NavStatus>::SharedPtr            nav_status_pub_;

  rclcpp_action::Server<NavAction>::SharedPtr action_server_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr  abort_srv_;
  rclcpp::Service<msgs::srv::SetTarget>::SharedPtr     set_target_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr   teleop_srv_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr           imu_sub_;
  rclcpp::Subscription<msgs::msg::PlannerEvent>::SharedPtr         planner_event_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr             global_path_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2D>::SharedPtr   aruco_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr             yolo_sub_;

  rclcpp::Client<msgs::srv::LatLonToENU>::SharedPtr latlon_client_;

  std::shared_ptr<GoalHandle> active_goal_handle_;
  std::shared_ptr<GoalHandle> queued_goal_handle_;

  rclcpp::TimerBase::SharedPtr status_timer_;
};

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
