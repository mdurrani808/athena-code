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
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "vision_msgs/msg/detection2_d.hpp"

#include <nlohmann/json.hpp>

#include "msgs/action/navigate_to_target.hpp"
#include "msgs/msg/active_target.hpp"
#include "msgs/msg/led_status.hpp"
#include "msgs/msg/local_planner_stuck.hpp"
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
  ARUCO_CONFIRM,
  ARUCO_APPROACH,
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
    case State::ARUCO_CONFIRM:     return "ARUCO_CONFIRM";
    case State::ARUCO_APPROACH:    return "ARUCO_APPROACH";
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
  bool skipped{false};
};

// ── Waypoint persistence (single JSON file, atomic save) ────────────────────

static const char* targetTypeStr(uint8_t t) {
  switch (t) {
    case 0: return "GNSS";
    case 1: return "ARUCO_1";
    case 2: return "ARUCO_2";
    case 3: return "OBJ_DETECT";
    default: return "UNKNOWN";
  }
}

static nlohmann::json entryToJson(const TargetEntry& e) {
  return nlohmann::json{
    {"id",          e.id},
    {"x",           e.x_m},
    {"y",           e.y_m},
    {"type",        targetTypeStr(e.target_type)},
    {"type_code",   e.target_type},
    {"goal_source", e.goal_source},
    {"tolerance",   e.tolerance_m},
    {"visited",     e.visited},
    {"skipped",     e.skipped},
  };
}

static std::optional<TargetEntry> jsonToEntry(const nlohmann::json& j) {
  try {
    TargetEntry e;
    e.id          = j.at("id").get<std::string>();
    e.x_m         = j.at("x").get<double>();
    e.y_m         = j.at("y").get<double>();
    e.target_type = j.at("type_code").get<uint8_t>();
    e.goal_source = j.at("goal_source").get<uint8_t>();
    e.tolerance_m = j.at("tolerance").get<double>();
    e.visited     = j.value("visited", false);
    e.skipped     = j.value("skipped", false);
    return e;
  } catch (const std::exception&) {
    return std::nullopt;
  }
}

static std::vector<TargetEntry> loadQueueFromFile(const std::string& path) {
  std::vector<TargetEntry> out;
  std::ifstream in(path);
  if (!in.is_open()) return out;
  try {
    nlohmann::json j;
    in >> j;
    if (!j.is_array()) return out;
    for (const auto& je : j) {
      if (auto e = jsonToEntry(je)) out.push_back(*e);
    }
  } catch (const std::exception&) {}
  return out;
}

static bool saveQueueToFile(const std::string& path,
                            const std::vector<TargetEntry>& q) {
  try {
    std::filesystem::path p(path);
    if (p.has_parent_path()) {
      std::filesystem::create_directories(p.parent_path());
    }
    const std::string tmp = path + ".tmp";
    {
      std::ofstream out(tmp, std::ios::trunc);
      if (!out.is_open()) return false;
      nlohmann::json arr = nlohmann::json::array();
      for (const auto& e : q) arr.push_back(entryToJson(e));
      out << arr.dump(2);
    }
    std::filesystem::rename(tmp, path);
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

static std::string expandTilde(const std::string& p) {
  if (p.empty() || p[0] != '~') return p;
  const char* home = std::getenv("HOME");
  if (!home) return p;
  return std::string(home) + p.substr(1);
}

struct MissionParams {
  double stop_angular_vel_threshold{0.05};
  double arrival_hold_time{1.0};
  double replan_distance_m{3.0};
  double spiral_timeout_s{120.0};
  double spiral_radius_m{15.0};
  double spiral_spacing_m{2.0};
  double spiral_angular_step{0.5};
  double spiral_waypoint_tolerance_m{2.0};
  int    aruco_confirm_frames{5};
  double aruco_max_age_s{0.5};
  double aruco_lost_timeout_s{1.0};
  // Visual-servo approach (image-based; no pose/solvePnP/TF).
  double aruco_servo_kp{0.8};            // angular gain on normalized bbox-center error
  double aruco_servo_linear_mps{0.4};    // forward speed while homing
  double aruco_servo_max_angular{0.6};   // clamp on commanded yaw rate (rad/s)
  double aruco_bbox_stop_frac{0.45};     // stop when bbox height / image height exceeds this
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
    declare_parameter("aruco_confirm_frames",         5);
    declare_parameter("aruco_max_age_s",              0.5);
    declare_parameter("aruco_lost_timeout_s",         1.0);
    declare_parameter("aruco_servo_kp",               0.8);
    declare_parameter("aruco_servo_linear_mps",       0.4);
    declare_parameter("aruco_servo_max_angular",      0.6);
    declare_parameter("aruco_bbox_stop_frac",         0.45);
    declare_parameter("aruco_detection_topic", std::string("/aruco_detection"));
    declare_parameter("camera_info_topic",     std::string("/zed/zed_node/left/camera_info"));
    declare_parameter("cmd_vel_topic",         std::string("/front_ackermann_controller/reference"));
    declare_parameter("yolo_detection_topic",  std::string("/yolo_detection"));
    declare_parameter("imu_topic",           std::string("/imu"));
    declare_parameter("planner_event_topic", std::string("/planner_event"));
    declare_parameter("global_path_topic",   std::string("/global_path"));
    declare_parameter("goal_pose_topic",     std::string("/goal_pose"));
    declare_parameter("nav_enabled_topic",   std::string("/nav_enabled"));
    declare_parameter("obstacle_avoidance_topic", std::string("/obstacle_avoidance_enabled"));
    declare_parameter("nav_mode_topic",      std::string("/nav_mode"));
    declare_parameter("active_target_topic", std::string("/active_target"));
    declare_parameter("nav_status_topic",    std::string("/nav_status"));
    declare_parameter("led_status_topic",    std::string("/led_status"));
    declare_parameter("local_planner_stuck_topic", std::string("/local_planner/stuck"));
    declare_parameter("waypoint_queue_topic",      std::string("/waypoint_queue"));
    declare_parameter("waypoint_cache_path",       std::string("~/.athena/waypoints.json"));
    declare_parameter("replan_throttle_s",         3.0);

    params_.stop_angular_vel_threshold  = get_parameter("stop_angular_vel_threshold").as_double();
    params_.arrival_hold_time           = get_parameter("arrival_hold_time").as_double();
    params_.replan_distance_m           = get_parameter("replan_distance_m").as_double();
    params_.spiral_timeout_s            = get_parameter("spiral_timeout_s").as_double();
    params_.spiral_radius_m             = get_parameter("spiral_radius_m").as_double();
    params_.spiral_spacing_m            = get_parameter("spiral_spacing_m").as_double();
    params_.spiral_angular_step         = get_parameter("spiral_angular_step").as_double();
    params_.spiral_waypoint_tolerance_m = get_parameter("spiral_waypoint_tolerance_m").as_double();
    params_.aruco_confirm_frames         = get_parameter("aruco_confirm_frames").as_int();
    params_.aruco_max_age_s              = get_parameter("aruco_max_age_s").as_double();
    params_.aruco_lost_timeout_s         = get_parameter("aruco_lost_timeout_s").as_double();
    params_.aruco_servo_kp               = get_parameter("aruco_servo_kp").as_double();
    params_.aruco_servo_linear_mps       = get_parameter("aruco_servo_linear_mps").as_double();
    params_.aruco_servo_max_angular      = get_parameter("aruco_servo_max_angular").as_double();
    params_.aruco_bbox_stop_frac         = get_parameter("aruco_bbox_stop_frac").as_double();

    const auto latlon_svc           = get_parameter("latlon_to_enu_service").as_string();
    aruco_detection_topic_          = get_parameter("aruco_detection_topic").as_string();
    const auto& aruco_detection_topic = aruco_detection_topic_;
    const auto camera_info_topic    = get_parameter("camera_info_topic").as_string();
    const auto cmd_vel_topic        = get_parameter("cmd_vel_topic").as_string();
    const auto yolo_detection_topic  = get_parameter("yolo_detection_topic").as_string();
    const auto imu_topic            = get_parameter("imu_topic").as_string();
    const auto planner_event_topic  = get_parameter("planner_event_topic").as_string();
    const auto global_path_topic    = get_parameter("global_path_topic").as_string();
    const auto goal_pose_topic      = get_parameter("goal_pose_topic").as_string();
    const auto nav_enabled_topic    = get_parameter("nav_enabled_topic").as_string();
    const auto avoidance_topic      = get_parameter("obstacle_avoidance_topic").as_string();
    const auto nav_mode_topic       = get_parameter("nav_mode_topic").as_string();
    const auto active_target_topic  = get_parameter("active_target_topic").as_string();
    const auto nav_status_topic     = get_parameter("nav_status_topic").as_string();
    const auto led_status_topic     = get_parameter("led_status_topic").as_string();
    const auto stuck_topic          = get_parameter("local_planner_stuck_topic").as_string();
    const auto waypoint_queue_topic = get_parameter("waypoint_queue_topic").as_string();
    waypoint_cache_path_            = expandTilde(get_parameter("waypoint_cache_path").as_string());
    replan_throttle_s_              = get_parameter("replan_throttle_s").as_double();

    queue_ = loadQueueFromFile(waypoint_cache_path_);
    RCLCPP_INFO(get_logger(), "Loaded %zu waypoints from %s",
      queue_.size(), waypoint_cache_path_.c_str());

    reentrant_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    goal_pub_          = create_publisher<geometry_msgs::msg::PoseStamped>(goal_pose_topic, 10);
    // Direct velocity command for the ArUco visual-servo approach. Published to
    // the same controller topic the local planner uses; the planner is held
    // silent (nav_enabled=false) during the servo so they never both drive.
    cmd_vel_pub_       = create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_topic, 10);
    nav_enabled_pub_   = create_publisher<std_msgs::msg::Bool>(
      nav_enabled_topic, rclcpp::QoS(1).reliable());
    nav_mode_pub_      = create_publisher<std_msgs::msg::String>(nav_mode_topic, 10);
    // Latched so the planner (a possibly-late/volatile subscriber) always gets
    // the current intent on connect; transient_local pub is compatible with the
    // planner's volatile sub for live updates.
    obstacle_avoidance_pub_ = create_publisher<std_msgs::msg::Bool>(
      avoidance_topic, rclcpp::QoS(1).reliable().transient_local());
    active_target_pub_ = create_publisher<msgs::msg::ActiveTarget>(active_target_topic, 10);
    nav_status_pub_    = create_publisher<msgs::msg::NavStatus>(
      nav_status_topic, rclcpp::QoS(1).reliable());
    led_status_pub_    = create_publisher<msgs::msg::LedStatus>(
      led_status_topic, rclcpp::QoS(1).reliable());
    waypoint_queue_pub_ = create_publisher<std_msgs::msg::String>(
      waypoint_queue_topic, rclcpp::QoS(1).reliable().transient_local());

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
        if (cmd.success) { publishNavEnabled(); publishNavMode(); publishLedStatus(); publishStatus(); }
      });

    set_target_srv_ = create_service<msgs::srv::SetTarget>(
      "~/set_target",
      [this](
        const msgs::srv::SetTarget::Request::SharedPtr req,
        msgs::srv::SetTarget::Response::SharedPtr res)
      { onSetTarget(req, res); },
      rmw_qos_profile_services_default,
      reentrant_group_);

    advance_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/advance",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr res)
      {
        std::lock_guard<std::mutex> lk(mutex_);
        auto cmd = onAdvance();
        res->success = cmd.success;
        res->message = cmd.message;
        if (cmd.success) { publishNavEnabled(); publishNavMode(); publishLedStatus(); publishStatus(); }
      });

    skip_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/skip",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr res)
      {
        std::lock_guard<std::mutex> lk(mutex_);
        auto cmd = onSkip();
        res->success = cmd.success;
        res->message = cmd.message;
        if (cmd.success) { publishNavEnabled(); publishNavMode(); publishLedStatus(); publishStatus(); }
      });

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
        if (cmd.success) { publishNavEnabled(); publishNavMode(); publishLedStatus(); publishStatus(); }
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
          publishLedStatus();
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
      [this](const vision_msgs::msg::Detection2D::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        onArucoDetection(msg);
      });

    // Camera intrinsics — only the image centre/size are needed for the
    // image-based servo. Latched, read once.
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        cam_width_       = static_cast<double>(msg->width);
        cam_height_      = static_cast<double>(msg->height);
        cam_cx_          = (msg->k[2] > 0.0) ? msg->k[2] : cam_width_ * 0.5;
        cam_info_ready_  = true;
      });

    yolo_sub_ = create_subscription<std_msgs::msg::Bool>(
      yolo_detection_topic, 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        if (msg->data) onDetection();
      });

    stuck_sub_ = create_subscription<msgs::msg::LocalPlannerStuck>(
      stuck_topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](const msgs::msg::LocalPlannerStuck::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        onPlannerStuck(*msg);
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
        publishLedStatus();
        publishAvoidanceState();
        publishStatus();
      });

    publishNavEnabled();
    publishNavMode();
    publishLedStatus();
    publishAvoidanceState();
    publishWaypointQueue();
    RCLCPP_INFO(get_logger(), "MissionExecutive ready — state: IDLE");
  }

private:
  // ── State machine ─────────────────────────────────────────────────────────

  bool isNavEnabled() const {
    // ARUCO_CONFIRM and ARUCO_APPROACH are deliberately excluded: dropping
    // /nav_enabled holds the local planner silent so it never fights the
    // visual-servo velocity commands (CONFIRM = stopped; APPROACH = servo drives).
    return state_ == State::NAVIGATING    ||
           state_ == State::ARRIVING      || state_ == State::RETURNING  ||
           state_ == State::SPIRAL_COVERAGE;
  }

  std::string getNavMode() const {
    switch (state_) {
      case State::TELEOP:        return "teleop";
      case State::NAVIGATING:
      case State::ARUCO_CONFIRM:
      case State::ARUCO_APPROACH:
      case State::ARRIVING:
      case State::RETURNING:
      case State::SPIRAL_COVERAGE: return "autonomous";
      default:                   return "stopped";
    }
  }

  bool isArucoTarget() const {
    return active_target_.has_value() &&
           (active_target_->target_type == 1 || active_target_->target_type == 2);
  }

  bool arucoTagVisible(double current_time_s) const {
    return aruco_last_detection_s_ > 0.0 &&
           (current_time_s - aruco_last_detection_s_) < params_.aruco_max_age_s;
  }

  void publishCmd(double linear, double angular) {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp    = now();
    cmd.header.frame_id = "base_link";
    cmd.twist.linear.x  = linear;
    cmd.twist.angular.z = angular;
    cmd_vel_pub_->publish(cmd);
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
            if (auto* e = findEntry(active_target_->id)) {
              e->visited = true;
              persistAndPublishQueue();
            }
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
    if (state_ == State::SPIRAL_COVERAGE && !isArucoTarget()) {
      detection_received_ = true;
      transition(State::SPIRAL_DONE);
    }
  }

  // Image-based ArUco homing. We use ONLY the 2D bounding box (centre pixel +
  // height) — no pose, no solvePnP, no TF. Centre the tag in frame and drive
  // forward; the tick() loop ends the approach when the box fills the frame.
  void onArucoDetection(const vision_msgs::msg::Detection2D::SharedPtr& msg) {
    if (!isArucoTarget()) return;
    if (msg->bbox.size_x <= 0.0 || msg->bbox.size_y <= 0.0) return;  // no usable box

    const double now_s = now().seconds();

    // Reset confirmation count if the tag was lost between frames.
    const bool gap_reset = (now_s - aruco_last_detection_s_ > params_.aruco_max_age_s);
    if (gap_reset) aruco_candidate_frames_ = 0;
    aruco_last_detection_s_ = now_s;
    aruco_candidate_frames_++;

    // Cache the latest box for the servo / heartbeat.
    aruco_bbox_cx_     = msg->bbox.center.position.x;
    aruco_bbox_height_ = msg->bbox.size_y;

    // Stop-and-confirm: the instant we first glimpse the tag while transiting or
    // searching, halt in place (ARUCO_CONFIRM drops /nav_enabled) and accumulate
    // the remaining confirmation frames so a single false positive can't trigger
    // an approach. If it's lost mid-confirm, tick() resumes the prior phase.
    if (state_ == State::NAVIGATING || state_ == State::SPIRAL_COVERAGE) {
      pre_confirm_state_ = state_;
      transition(State::ARUCO_CONFIRM);
      publishNavEnabled();  // halt the planner immediately, don't wait for the timer
      publishCmd(0.0, 0.0); // and actively brake so the tag stays in the FOV to confirm
    }

    if (aruco_candidate_frames_ < params_.aruco_confirm_frames) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "[ARUCO] tag seen — holding to confirm %d/%d frames%s",
        aruco_candidate_frames_, params_.aruco_confirm_frames,
        gap_reset ? " (restarted after gap)" : "");
      return;
    }

    // Confirmed — lock on and start the visual-servo approach.
    if (state_ == State::ARUCO_CONFIRM) {
      transition(State::ARUCO_APPROACH);
      publishNavEnabled();  // keep the planner silent; the servo owns cmd_vel now
      RCLCPP_INFO(get_logger(), "[ARUCO] confirmed — starting visual-servo approach");
    }

    // Drive the servo at camera rate while approaching.
    if (state_ == State::ARUCO_APPROACH) driveVisualServo();
  }

  // Centre the tag (bbox centre → image centre) and drive forward. Sets the
  // arrival flag once the box fills enough of the frame; tick() finalizes.
  void driveVisualServo() {
    if (!cam_info_ready_ || cam_cx_ <= 0.0 || cam_height_ <= 0.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "[ARUCO] no camera_info yet — cannot visual-servo");
      publishCmd(0.0, 0.0);
      return;
    }

    // Arrival: the box is large enough → we are right in front of the post.
    const double height_frac = aruco_bbox_height_ / cam_height_;
    if (height_frac >= params_.aruco_bbox_stop_frac) {
      publishCmd(0.0, 0.0);
      aruco_arrived_ = true;  // tick() turns this into STOPPED_AT_TARGET
      return;
    }

    // Horizontal error, normalized to [-1, 1]. Image +x is to the right; turning
    // right is a negative yaw rate, so angular = -kp * err.
    const double err = (aruco_bbox_cx_ - cam_cx_) / cam_cx_;
    double angular = -params_.aruco_servo_kp * err;
    angular = std::clamp(angular,
                         -params_.aruco_servo_max_angular,
                          params_.aruco_servo_max_angular);
    // Slow forward speed when the tag is off-centre so we straighten before
    // closing distance (and the tag stays in frame).
    const double linear = params_.aruco_servo_linear_mps *
                          std::clamp(1.0 - std::fabs(err), 0.2, 1.0);
    publishCmd(linear, angular);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "[ARUCO] servo: err=%+.2f height=%.0f%% lin=%.2f ang=%+.2f",
      err, height_frac * 100.0, linear, angular);
  }

  TargetEntry* findEntry(const std::string& id) {
    for (auto& e : queue_) if (e.id == id) return &e;
    return nullptr;
  }

  // Re-sort the not-yet-acted-on entries (excluding the currently active one)
  // by distance from current robot pose. Visited/skipped entries keep their
  // slot positions so the GUI sees a stable history list.
  void sortPending() {
    if (!robot_pose_.has_value()) return;
    const double rx = robot_pose_->x;
    const double ry = robot_pose_->y;
    std::vector<size_t> idx;
    idx.reserve(queue_.size());
    for (size_t i = 0; i < queue_.size(); ++i) {
      const auto& e = queue_[i];
      if (e.visited || e.skipped) continue;
      if (active_target_.has_value() && active_target_->id == e.id) continue;
      idx.push_back(i);
    }
    if (idx.size() < 2) return;
    std::vector<TargetEntry> entries;
    entries.reserve(idx.size());
    for (auto i : idx) entries.push_back(queue_[i]);
    std::sort(entries.begin(), entries.end(),
      [rx, ry](const TargetEntry& a, const TargetEntry& b) {
        return std::hypot(a.x_m - rx, a.y_m - ry) <
               std::hypot(b.x_m - rx, b.y_m - ry);
      });
    for (size_t k = 0; k < idx.size(); ++k) queue_[idx[k]] = entries[k];
  }

  std::optional<TargetEntry> popNextPending() {
    for (const auto& e : queue_) {
      if (!e.visited && !e.skipped) {
        if (active_target_.has_value() && active_target_->id == e.id) continue;
        return e;
      }
    }
    return std::nullopt;
  }

  std::optional<TargetEntry> lastVisitedBefore(const std::string& current_id) {
    std::optional<TargetEntry> last;
    for (const auto& e : queue_) {
      if (e.id == current_id) break;
      if (e.visited) last = e;
    }
    return last;
  }

  void persistAndPublishQueue() {
    if (!waypoint_cache_path_.empty()) {
      if (!saveQueueToFile(waypoint_cache_path_, queue_)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Failed to persist waypoint cache to '%s'", waypoint_cache_path_.c_str());
      }
    }
    publishWaypointQueue();
  }

  CommandResult setTarget(const TargetEntry& entry) {
    if (auto* existing = findEntry(entry.id)) {
      const bool was_terminal = existing->visited || existing->skipped;
      *existing = entry;
      // Update of an in-progress entry keeps its slot; only pending re-sort.
      if (!was_terminal) sortPending();
    } else {
      queue_.push_back(entry);
      sortPending();
    }
    persistAndPublishQueue();
    return {true, "Target '" + entry.id + "' registered"};
  }

  StartNavResult startNav(const std::string& target_id,
                          const std::optional<TargetEntry>& inline_target,
                          bool is_return) {
    StartNavResult res;
    TargetEntry entry;
    if (!target_id.empty()) {
      auto* found = findEntry(target_id);
      if (!found) {
        return {false, "Unknown target_id: " + target_id};
      }
      if (is_return && !found->visited) {
        return {false, "Target not yet visited — cannot RETURN"};
      }
      entry = *found;
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

    active_target_          = entry;
    is_return_              = is_return;
    aruco_candidate_frames_ = 0;
    aruco_last_detection_s_ = 0.0;
    aruco_arrived_          = false;
    aruco_bbox_height_      = 0.0;
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

  CommandResult onAdvance() {
    const bool advanceable = (state_ == State::IDLE ||
                              state_ == State::STOPPED_AT_TARGET ||
                              state_ == State::STOPPED_AT_RETURN ||
                              state_ == State::SPIRAL_DONE);
    if (!advanceable) {
      return {false, "Cannot advance from state " + stateToStr(state_)};
    }
    auto next = popNextPending();
    if (!next.has_value()) {
      return {false, "No pending waypoints"};
    }
    auto nav = startNav(next->id, std::nullopt, /*is_return=*/false);
    if (!nav.accepted) return {false, nav.message};
    if (nav.publish_goal) {
      publishGoal(nav.goal_to_publish);
      publishActiveTarget();
    }
    persistAndPublishQueue();
    return {true, "Advanced to '" + next->id + "'"};
  }

  CommandResult onSkip() {
    if (!active_target_.has_value()) {
      return {false, "No active waypoint to skip"};
    }
    if (auto* e = findEntry(active_target_->id)) {
      e->skipped = true;
    }
    active_target_.reset();
    transition(State::IDLE);
    persistAndPublishQueue();
    return {true, "Skipped — call ~/advance for next waypoint"};
  }

  void onPlannerStuck(const msgs::msg::LocalPlannerStuck& msg) {
    const bool rising = msg.stuck && !prev_stuck_;
    prev_stuck_ = msg.stuck;
    if (!rising) return;
    if (!active_target_.has_value()) return;
    if (state_ != State::NAVIGATING && state_ != State::RETURNING) return;
    const double now_s = now().seconds();
    if (now_s - last_replan_pub_s_ < replan_throttle_s_) return;
    last_replan_pub_s_ = now_s;
    publishGoal({active_target_->x_m, active_target_->y_m, 0.0});
    RCLCPP_WARN(get_logger(),
      "[REPLAN] /local_planner/stuck rising edge — re-publishing goal for '%s'",
      active_target_->id.c_str());
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

  std::vector<Pose2D> generateSpiralWaypoints(const Pose2D& center) {
    std::vector<Pose2D> waypoints;
    const double a = params_.spiral_spacing_m / (2.0 * M_PI);
    if (a <= 0.0) return waypoints;
    // Drive to the centre first — the point where the tag/object is expected —
    // then spiral outward from it.
    waypoints.push_back({center.x, center.y, 0.0});
    const double max_angle   = params_.spiral_radius_m / a;
    const double min_start_r = 1.5;
    const double yaw0        = center.yaw;
    for (double theta = 0.0; theta <= max_angle; theta += params_.spiral_angular_step) {
      const double r = min_start_r + a * theta;
      if (r > params_.spiral_radius_m) break;
      waypoints.push_back({
        center.x + r * std::cos(yaw0 + theta + M_PI_2),
        center.y + r * std::sin(yaw0 + theta + M_PI_2),
        0.0
      });
    }
    return waypoints;
  }

  void startSpiralCoverage(double current_time_s) {
    // Centre the spiral on the TARGET point (where the tag/object should be),
    // not on wherever the rover stopped within the GPS arrival tolerance. The
    // first spiral waypoint is the target point itself, so the rover drives
    // there first and then expands outward. (If a tag is spotted at any point,
    // tick() step 1a preempts the spiral and switches to ARUCO_APPROACH.)
    Pose2D center;
    if (active_target_.has_value()) {
      center.x   = active_target_->x_m;
      center.y   = active_target_->y_m;
      center.yaw = robot_pose_.has_value() ? robot_pose_->yaw : 0.0;
    } else if (robot_pose_.has_value()) {
      center = robot_pose_.value();
    } else {
      transition(State::SPIRAL_DONE);
      return;
    }
    spiral_waypoints_    = generateSpiralWaypoints(center);
    spiral_waypoint_idx_ = 0;
    spiral_start_time_s_ = current_time_s;
    detection_received_  = false;
    if (spiral_waypoints_.empty()) transition(State::SPIRAL_DONE);
  }

  // Advance spiral_waypoint_idx_ past every waypoint the rover is already within
  // tolerance of, then return the next not-yet-reached waypoint as the goal.
  // Collapsing the cluster in one step (instead of one index per tick) is what
  // keeps the published goal genuinely ahead of the rover: spiral waypoints are
  // spaced far closer than spiral_waypoint_tolerance_m, so advancing one-at-a-
  // time republishes a goal that circles the centre and the rover never commits
  // to a heading. Returns false when the spiral is exhausted.
  bool nextSpiralGoal(Pose2D& goal_out) {
    if (spiral_waypoints_.empty()) return false;
    if (robot_pose_.has_value()) {
      while (spiral_waypoint_idx_ < spiral_waypoints_.size()) {
        const auto& wp = spiral_waypoints_[spiral_waypoint_idx_];
        if (std::hypot(robot_pose_->x - wp.x, robot_pose_->y - wp.y)
            >= params_.spiral_waypoint_tolerance_m) {
          break;
        }
        ++spiral_waypoint_idx_;
      }
    }
    if (spiral_waypoint_idx_ >= spiral_waypoints_.size()) return false;
    goal_out = spiral_waypoints_[spiral_waypoint_idx_];
    return true;
  }

  TickResult tick(double current_time_s) {
    TickResult res;

    // ArUco debug heartbeat — one line at 1 Hz whenever an ArUco post is the
    // active target: state, confirm progress, whether the tag is currently in
    // view, the normalized horizontal error, and how much of the frame it fills.
    if (isArucoTarget()) {
      const double det_age = (aruco_last_detection_s_ > 0.0)
        ? current_time_s - aruco_last_detection_s_ : -1.0;
      const double err = (cam_info_ready_ && cam_cx_ > 0.0)
        ? (aruco_bbox_cx_ - cam_cx_) / cam_cx_ : 0.0;
      const double hfrac = (cam_height_ > 0.0) ? aruco_bbox_height_ / cam_height_ : 0.0;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "[ARUCO] state=%s id=%s frames=%d/%d visible=%d last_det=%.1fs err=%+.2f height=%.0f%%",
        stateToStr(state_).c_str(), active_target_->id.c_str(),
        aruco_candidate_frames_, params_.aruco_confirm_frames,
        static_cast<int>(arucoTagVisible(current_time_s)), det_age, err, hfrac * 100.0);
    }

    // 1a. ArUco visual-servo approach. The servo loop (centre + drive) runs in
    //     onArucoDetection at camera rate; here we only finalize arrival and
    //     handle a lost tag. The rover stops (nav_enabled is dropped in this
    //     state, and we publish zero) whenever the tag is not currently in view.
    if (state_ == State::ARUCO_APPROACH) {
      if (aruco_arrived_) {
        publishCmd(0.0, 0.0);
        transition(State::STOPPED_AT_TARGET);
        res.action_finished = true;
        res.action_success  = true;
        res.action_message  = "Reached ArUco post (tag filled frame)";
        return res;
      }
      if (!arucoTagVisible(current_time_s)) {
        publishCmd(0.0, 0.0);  // do not drive blind between frames
        if (current_time_s - aruco_last_detection_s_ >= params_.aruco_lost_timeout_s) {
          const double hfrac = (cam_height_ > 0.0) ? aruco_bbox_height_ / cam_height_ : 0.0;
          if (hfrac >= 0.5 * params_.aruco_bbox_stop_frac) {
            // Already close when it left the FOV → count it as reached.
            transition(State::STOPPED_AT_TARGET);
            res.action_finished = true;
            res.action_success  = true;
            res.action_message  = "Reached ArUco post (tag left view at close range)";
            return res;
          }
          // Lost while still far — resume the spiral search.
          RCLCPP_WARN(get_logger(),
            "[ARUCO] tag lost mid-approach while still far (height=%.0f%%) — resuming spiral",
            hfrac * 100.0);
          transition(State::SPIRAL_COVERAGE);
          startSpiralCoverage(current_time_s);
          Pose2D spiral_goal;
          if (state_ == State::SPIRAL_COVERAGE) {
            if (nextSpiralGoal(spiral_goal)) {
              res.publish_goal    = true;
              res.goal_to_publish = spiral_goal;
            } else {
              transition(State::SPIRAL_DONE);
            }
          }
        }
      }
      return res;  // tag visible → the servo callback is driving
    }

    // 1c. Stop-and-confirm hold. Actively brake every tick (don't rely on the
    //     planner's single stop) so the rover holds still and the tag stays in
    //     the FOV long enough to accumulate confirmation frames. onArucoDetection
    //     promotes us to ARUCO_APPROACH once confirmed; here we keep zero velocity
    //     and detect loss to resume the prior search/transit phase.
    if (state_ == State::ARUCO_CONFIRM) {
      publishCmd(0.0, 0.0);
      if (current_time_s - aruco_last_detection_s_ > params_.aruco_max_age_s) {
        RCLCPP_INFO(get_logger(),
          "[ARUCO] tag lost before confirming (%d/%d) — resuming %s",
          aruco_candidate_frames_, params_.aruco_confirm_frames,
          stateToStr(pre_confirm_state_).c_str());
        aruco_candidate_frames_ = 0;
        transition(pre_confirm_state_);
        if (pre_confirm_state_ == State::SPIRAL_COVERAGE &&
            spiral_waypoint_idx_ < spiral_waypoints_.size()) {
          res.publish_goal    = true;
          res.goal_to_publish = spiral_waypoints_[spiral_waypoint_idx_];
        } else if (active_target_.has_value()) {
          res.publish_goal    = true;
          res.goal_to_publish = {active_target_->x_m, active_target_->y_m, 0.0};
        }
      }
      return res;  // otherwise hold position and keep accumulating frames
    }

    // 1b. GPS arrival check
    if ((state_ == State::NAVIGATING || state_ == State::RETURNING) &&
        active_target_.has_value()) {
      const double d = getDistToGoal();
      if (d >= 0.0) {
        if (isArucoTarget()) {
          // Drive all the way TO the given GPS point, then spiral outward from
          // it to search for the tag. The spiral is the deterministic search
          // phase — NOT a fallback triggered by "arrived without tag lock". A
          // confirmed tag at any point preempts to ARUCO_APPROACH via step 1a
          // above, regardless of state.
          if (d < params_.spiral_waypoint_tolerance_m) {
            transition(State::SPIRAL_COVERAGE);
            startSpiralCoverage(current_time_s);
            Pose2D spiral_goal;
            if (state_ == State::SPIRAL_COVERAGE) {
              if (nextSpiralGoal(spiral_goal)) {
                res.publish_goal    = true;
                res.goal_to_publish = spiral_goal;
              } else {
                transition(State::SPIRAL_DONE);
              }
            }
          }
        } else if (d < active_target_->tolerance_m) {
          transition(State::ARRIVING);
        }
      }
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
        const size_t prev_idx = spiral_waypoint_idx_;
        Pose2D goal;
        if (!nextSpiralGoal(goal)) {
          transition(State::SPIRAL_DONE);
        } else if (spiral_waypoint_idx_ != prev_idx) {
          // Only republish (and trigger a global replan) when the target
          // waypoint actually changed.
          res.publish_goal    = true;
          res.goal_to_publish = goal;
        }
      }
    }

    // 4. Action result resolution
    if (state_ == State::STOPPED_AT_TARGET) {
      // type 3 = OBJECT (needs spiral+YOLO); ArUco posts (1,2) finish via aruco_stop above
      const bool needs_spiral = active_target_.has_value() &&
        active_target_->target_type == 3;
      if (needs_spiral) {
        transition(State::SPIRAL_COVERAGE);
        startSpiralCoverage(current_time_s);
        Pose2D spiral_goal;
        if (state_ == State::SPIRAL_COVERAGE) {
          if (nextSpiralGoal(spiral_goal)) {
            res.publish_goal    = true;
            res.goal_to_publish = spiral_goal;
          } else {
            transition(State::SPIRAL_DONE);
          }
        }
      } else {
        res.action_finished = true;
        res.action_success  = true;
        res.action_message  = "Arrived at target";
      }
    } else if (state_ == State::SPIRAL_DONE) {
      res.action_finished = true;
      if (isArucoTarget()) {
        // A confirmed tag preempts to ARUCO_APPROACH (step 1a) and finishes at
        // STOPPED_AT_TARGET, so reaching SPIRAL_DONE means the tag was never
        // found within the search radius/timeout — that is a failure.
        res.action_success = false;
        res.action_message = "ArUco tag not found during spiral search";
      } else {
        res.action_success = true;
        res.action_message = detection_received_
          ? "Detection found during spiral coverage"
          : "Spiral coverage complete (timeout or all waypoints visited)";
      }
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

  // Obstacle avoidance should be ON for normal navigation and object-detection
  // (type 3) spirals, but OFF while searching for / homing on an ArUco post:
  // the post reads as an obstacle in the laserscan, so avoidance would steer the
  // rover away from the very tag it must drive up to.
  bool desiredAvoidanceEnabled() const {
    if (isArucoTarget() &&
        (state_ == State::SPIRAL_COVERAGE ||
         state_ == State::ARUCO_CONFIRM ||
         state_ == State::ARUCO_APPROACH)) {
      return false;
    }
    return true;
  }

  void publishAvoidanceState() {
    const bool want = desiredAvoidanceEnabled();
    if (last_avoidance_enabled_.has_value() && last_avoidance_enabled_.value() == want) {
      return;  // publish only on change
    }
    std_msgs::msg::Bool msg;
    msg.data = want;
    obstacle_avoidance_pub_->publish(msg);
    last_avoidance_enabled_ = want;
    RCLCPP_INFO(get_logger(), "[AVOIDANCE] obstacle avoidance %s%s",
      want ? "ENABLED" : "DISABLED",
      want ? "" : " — ArUco approach (post would read as an obstacle)");
  }

  void publishWaypointQueue() {
    if (!waypoint_queue_pub_) return;
    nlohmann::json arr = nlohmann::json::array();
    for (const auto& e : queue_) {
      auto je = entryToJson(e);
      const bool is_active = active_target_.has_value() &&
                             active_target_->id == e.id &&
                             !e.visited && !e.skipped;
      std::string status = "PENDING";
      if (e.skipped)       status = "SKIPPED";
      else if (e.visited)  status = "VISITED";
      else if (is_active)  status = "ACTIVE";
      je["status"] = status;
      arr.push_back(je);
    }
    std_msgs::msg::String s;
    s.data = arr.dump();
    waypoint_queue_pub_->publish(s);
  }

  void publishLedStatus() {
    msgs::msg::LedStatus led;
    switch (state_) {
      case State::NAVIGATING:
      case State::ARUCO_CONFIRM:
      case State::ARUCO_APPROACH:
      case State::ARRIVING:
      case State::RETURNING:
      case State::SPIRAL_COVERAGE:
      case State::ABORTING:
        // Red — autonomous operation
        led.cmd = msgs::msg::LedStatus::CMD_SOLID;
        led.r   = 255; led.g = 0; led.b = 0;
        break;
      case State::TELEOP:
        // Blue — teleoperation
        led.cmd = msgs::msg::LedStatus::CMD_SOLID;
        led.r   = 0; led.g = 0; led.b = 255;
        break;
      case State::STOPPED_AT_TARGET:
      case State::STOPPED_AT_RETURN:
      case State::SPIRAL_DONE:
        // Flashing green — successful arrival
        led.cmd   = msgs::msg::LedStatus::CMD_FLASH;
        led.r     = 0; led.g = 255; led.b = 0;
        led.param = 20;  // 2 Hz
        break;
      default:
        led.cmd = msgs::msg::LedStatus::CMD_OFF;
        break;
    }
    led_status_pub_->publish(led);
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
      // Guard against a non-active handle (e.g. already terminal, or shutting
      // down) — succeed()/abort() on such a handle throws.
      if (active_goal_handle_->is_active()) {
        auto result     = std::make_shared<NavAction::Result>();
        result->success = res.action_success;
        result->message = res.action_message;
        if (res.action_success) active_goal_handle_->succeed(result);
        else                    active_goal_handle_->abort(result);
      }
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
      publishLedStatus();
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

  std::vector<TargetEntry> queue_;
  std::string waypoint_cache_path_;
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

  // ArUco visual-servo approach state (image-based; no pose/TF/solvePnP)
  std::string aruco_detection_topic_;
  State  pre_confirm_state_{State::SPIRAL_COVERAGE};
  int    aruco_candidate_frames_{0};
  double aruco_last_detection_s_{0.0};
  double aruco_bbox_cx_{0.0};       // latest bbox centre x (pixels)
  double aruco_bbox_height_{0.0};   // latest bbox height (pixels)
  bool   aruco_arrived_{false};     // bbox filled the frame → at the post
  // Camera intrinsics (image centre / size only)
  bool   cam_info_ready_{false};
  double cam_cx_{0.0};
  double cam_width_{0.0};
  double cam_height_{0.0};

  // ── ROS handles ───────────────────────────────────────────────────────────

  rclcpp::CallbackGroup::SharedPtr reentrant_group_;

  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr             nav_enabled_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           nav_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr            obstacle_avoidance_pub_;
  std::optional<bool>                                          last_avoidance_enabled_;
  rclcpp::Publisher<msgs::msg::ActiveTarget>::SharedPtr         active_target_pub_;
  rclcpp::Publisher<msgs::msg::NavStatus>::SharedPtr            nav_status_pub_;
  rclcpp::Publisher<msgs::msg::LedStatus>::SharedPtr            led_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           waypoint_queue_pub_;

  rclcpp_action::Server<NavAction>::SharedPtr action_server_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr  abort_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr  advance_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr  skip_srv_;
  rclcpp::Service<msgs::srv::SetTarget>::SharedPtr     set_target_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr   teleop_srv_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr           imu_sub_;
  rclcpp::Subscription<msgs::msg::PlannerEvent>::SharedPtr         planner_event_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr             global_path_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2D>::SharedPtr   aruco_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr    camera_info_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr             yolo_sub_;
  rclcpp::Subscription<msgs::msg::LocalPlannerStuck>::SharedPtr    stuck_sub_;

  // Replan-on-stuck state
  bool   prev_stuck_{false};
  double last_replan_pub_s_{0.0};
  double replan_throttle_s_{3.0};

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
