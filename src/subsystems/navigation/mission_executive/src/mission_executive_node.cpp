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

#include "mission_executive/mission_executive_algo.hpp"

using namespace std::chrono_literals;

class MissionExecutive : public rclcpp::Node
{
public:
  using NavAction  = msgs::action::NavigateToTarget;
  using GoalHandle = rclcpp_action::ServerGoalHandle<NavAction>;

  explicit MissionExecutive(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("mission_executive", options)
  {
    // Behaviour parameters
    declare_parameter("stop_angular_vel_threshold", 0.05);  // rad/s — IMU gyro magnitude
    declare_parameter("arrival_hold_time",           1.0);
    declare_parameter("replan_distance_m",           3.0);
    declare_parameter("latlon_to_enu_service",
      std::string("/gps_pose_publisher/latlon_to_enu"));

    // Spiral coverage parameters
    declare_parameter("spiral_timeout_s",            120.0);
    declare_parameter("spiral_radius_m",             15.0);
    declare_parameter("spiral_spacing_m",            2.0);
    declare_parameter("spiral_angular_step",         0.5);
    declare_parameter("spiral_waypoint_tolerance_m", 2.0);

    // Detection topics
    declare_parameter("aruco_detection_topic", std::string("/aruco_loc"));
    declare_parameter("yolo_detection_topic",  std::string("/yolo_detection"));

    // Input topics
    declare_parameter("imu_topic",           std::string("/imu"));
    declare_parameter("planner_event_topic", std::string("/planner_event"));
    declare_parameter("global_path_topic",   std::string("/global_path"));

    // Output topics
    declare_parameter("goal_pose_topic",     std::string("/goal_pose"));
    declare_parameter("nav_enabled_topic",   std::string("/nav_enabled"));
    declare_parameter("nav_mode_topic",      std::string("/nav_mode"));
    declare_parameter("active_target_topic", std::string("/active_target"));
    declare_parameter("nav_status_topic",    std::string("/nav_status"));

    mission_executive::MissionParams p;
    p.stop_angular_vel_threshold = get_parameter("stop_angular_vel_threshold").as_double();
    p.arrival_hold_time          = get_parameter("arrival_hold_time").as_double();
    p.replan_distance_m          = get_parameter("replan_distance_m").as_double();
    p.spiral_timeout_s           = get_parameter("spiral_timeout_s").as_double();
    p.spiral_radius_m            = get_parameter("spiral_radius_m").as_double();
    p.spiral_spacing_m           = get_parameter("spiral_spacing_m").as_double();
    p.spiral_angular_step        = get_parameter("spiral_angular_step").as_double();
    p.spiral_waypoint_tolerance_m= get_parameter("spiral_waypoint_tolerance_m").as_double();
    
    algo_.setParams(p);

    const auto latlon_svc         = get_parameter("latlon_to_enu_service").as_string();

    const auto aruco_detection_topic = get_parameter("aruco_detection_topic").as_string();
    const auto yolo_detection_topic  = get_parameter("yolo_detection_topic").as_string();

    const auto imu_topic           = get_parameter("imu_topic").as_string();
    const auto planner_event_topic = get_parameter("planner_event_topic").as_string();
    const auto global_path_topic   = get_parameter("global_path_topic").as_string();
    const auto goal_pose_topic     = get_parameter("goal_pose_topic").as_string();
    const auto nav_enabled_topic   = get_parameter("nav_enabled_topic").as_string();
    const auto nav_mode_topic      = get_parameter("nav_mode_topic").as_string();
    const auto active_target_topic = get_parameter("active_target_topic").as_string();
    const auto nav_status_topic    = get_parameter("nav_status_topic").as_string();

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

    abort_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/abort",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr res)
      {
        std::lock_guard<std::mutex> lk(mutex_);
        auto cmd_res = algo_.abort();
        res->success = cmd_res.success;
        res->message = cmd_res.message;
        if (cmd_res.success) {
          publishNavEnabled(algo_.isNavEnabled());
          publishNavMode(algo_.getNavMode());
          publishStatus();
        }
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
        auto cmd_res = algo_.setTeleop(req->data);
        res->success = cmd_res.success;
        res->message = cmd_res.message;
        if (cmd_res.success) {
          publishNavEnabled(algo_.isNavEnabled());
          publishNavMode(algo_.getNavMode());
          publishStatus();
        }
      });

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        const auto & w = msg->angular_velocity;
        double angular_vel = std::sqrt(w.x * w.x + w.y * w.y + w.z * w.z);
        double current_time = now().seconds();
        algo_.updateImu(angular_vel, current_time);
        
        // This could trigger a transition to STOPPED_AT_TARGET, which then needs action server response
        // but we handle action results in tick()
      });

    planner_event_sub_ = create_subscription<msgs::msg::PlannerEvent>(
      planner_event_topic, 10,
      [this](const msgs::msg::PlannerEvent::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        algo_.updatePlannerEvent(msg->event);
        if (msg->event == msgs::msg::PlannerEvent::PLAN_FAILED) {
          algo_.onPlanFailed();
          publishNavEnabled(algo_.isNavEnabled());
          publishNavMode(algo_.getNavMode());
          publishStatus();
        }
      });

    auto transient_qos = rclcpp::QoS(1).transient_local();
    global_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      global_path_topic, transient_qos,
      [this](const nav_msgs::msg::Path::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        std::vector<mission_executive::Pose2D> path;
        path.reserve(msg->poses.size());
        for (const auto& p : msg->poses) {
          path.push_back({p.pose.position.x, p.pose.position.y, quaternionToYaw(p.pose.orientation)});
        }
        algo_.updateGlobalPath(path);
      });

    aruco_sub_ = create_subscription<vision_msgs::msg::Detection2D>(
      aruco_detection_topic, 10,
      [this](const vision_msgs::msg::Detection2D::SharedPtr) {
        std::lock_guard<std::mutex> lk(mutex_);
        algo_.onDetection();
      });

    yolo_sub_ = create_subscription<std_msgs::msg::Bool>(
      yolo_detection_topic, 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        if (msg->data) {
          algo_.onDetection();
        }
      });

    status_timer_ = create_wall_timer(
      500ms,
      [this]() {
        std::lock_guard<std::mutex> lk(mutex_);
        refreshPoseFromTF();
        
        auto res = algo_.tick(now().seconds());
        
        if (res.publish_goal) {
          geometry_msgs::msg::PoseStamped p;
          p.header.frame_id = "map";
          p.header.stamp = now();
          p.pose.position.x = res.goal_to_publish.x;
          p.pose.position.y = res.goal_to_publish.y;
          p.pose.orientation.w = 1.0;
          goal_pub_->publish(p);
        }

        checkActionResult(res);

        publishNavEnabled(algo_.isNavEnabled());
        publishNavMode(algo_.getNavMode());
        publishStatus();
      });

    publishNavEnabled(false);
    publishNavMode("stopped");

    RCLCPP_INFO(get_logger(), "MissionExecutive ready — state: IDLE");
  }

private:
  void publishNavEnabled(bool enabled) {
    std_msgs::msg::Bool msg;
    msg.data = enabled;
    nav_enabled_pub_->publish(msg);
  }

  void publishNavMode(const std::string& mode) {
    std_msgs::msg::String msg;
    msg.data = mode;
    nav_mode_pub_->publish(msg);
  }

  void publishActiveTarget() {
    auto target = algo_.getActiveTarget();
    if (!target.has_value()) return;
    msgs::msg::ActiveTarget at;
    at.target_id   = target->id;
    at.target_type = target->target_type;
    at.tolerance_m = target->tolerance_m;
    
    at.goal_enu.header.frame_id = "map";
    at.goal_enu.header.stamp = now();
    at.goal_enu.pose.position.x = target->x_m;
    at.goal_enu.pose.position.y = target->y_m;
    at.goal_enu.pose.orientation.w = 1.0;

    at.goal_source = target->goal_source;
    at.status      = mission_executive::stateToStr(algo_.getState());
    active_target_pub_->publish(at);
  }

  void publishStatus() {
    msgs::msg::NavStatus s;
    s.state = mission_executive::stateToStr(algo_.getState());
    auto target = algo_.getActiveTarget();
    if (target.has_value()) {
      s.active_target_id   = target->id;
      s.active_target_type = target->target_type;
      s.goal_source        = target->goal_source;
    }
    s.distance_to_goal_m  = algo_.getDistToGoal();
    s.cross_track_error_m = algo_.getCrossTrackError();
    s.heading_error_rad   = algo_.getHeadingError();
    s.robot_speed_mps     = algo_.getImuAngularVel();
    s.is_return           = algo_.isReturn();
    s.last_planner_event  = algo_.getLastPlannerEvent();
    nav_status_pub_->publish(s);
  }

  static double quaternionToYaw(const geometry_msgs::msg::Quaternion & q) {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  void refreshPoseFromTF() {
    try {
      const auto tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
      mission_executive::Pose2D pose;
      pose.x = tf.transform.translation.x;
      pose.y = tf.transform.translation.y;
      pose.yaw = quaternionToYaw(tf.transform.rotation);
      algo_.updateRobotPose(pose);
    } catch (const tf2::TransformException &) {
    }
  }

  void checkActionResult(const mission_executive::TickResult& res) {
    if (!active_goal_handle_) return;

    if (active_goal_handle_->is_canceling()) {
      auto result = std::make_shared<NavAction::Result>();
      result->success = false;
      result->message = "Cancelled";
      active_goal_handle_->canceled(result);
      active_goal_handle_ = nullptr;
      algo_.cancelNav();
      return;
    }

    if (res.action_finished) {
      auto result = std::make_shared<NavAction::Result>();
      result->success = res.action_success;
      result->message = res.action_message;

      if (res.action_success) {
        active_goal_handle_->succeed(result);
      } else {
        active_goal_handle_->abort(result);
      }
      active_goal_handle_ = nullptr;
    } else {
      auto fb = std::make_shared<NavAction::Feedback>();
      fb->distance_to_goal_m  = algo_.getDistToGoal();
      fb->cross_track_error_m = algo_.getCrossTrackError();
      fb->state               = mission_executive::stateToStr(algo_.getState());
      active_goal_handle_->publish_feedback(fb);
    }

    if (res.start_queued_goal) {
      active_goal_handle_ = queued_goal_handle_;
      queued_goal_handle_ = nullptr;
      publishActiveTarget();
    }
  }

  rclcpp_action::GoalResponse handleGoal(
    std::shared_ptr<const NavAction::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(std::shared_ptr<GoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(std::shared_ptr<GoalHandle> goal_handle) {
    const auto goal = goal_handle->get_goal();

    std::optional<mission_executive::TargetEntry> inline_target;
    
    if (goal->target_id.empty()) {
      mission_executive::TargetEntry entry;
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
        auto resp = future.get();
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

      auto cmd_res = algo_.startNav(goal->target_id, inline_target, goal->is_return);
      
      if (!cmd_res.accepted) {
        auto res = std::make_shared<NavAction::Result>();
        res->success = false;
        res->message = cmd_res.message;
        goal_handle->abort(res);
        return;
      }

      if (algo_.hasQueuedGoal() && algo_.getState() == mission_executive::State::ABORTING) {
        if (queued_goal_handle_ && queued_goal_handle_->is_active()) {
          auto old_res = std::make_shared<NavAction::Result>();
          old_res->success = false;
          old_res->message = "Preempted by newer queued goal";
          queued_goal_handle_->abort(old_res);
        }
        queued_goal_handle_ = goal_handle;
        return;
      }

      if (cmd_res.preempted_old) {
        if (active_goal_handle_ && active_goal_handle_->is_active()) {
          auto old_res = std::make_shared<NavAction::Result>();
          old_res->success = false;
          old_res->message = "Preempted by new goal";
          active_goal_handle_->abort(old_res);
          active_goal_handle_ = nullptr;
        }
      }

      active_goal_handle_ = goal_handle;

      if (cmd_res.publish_goal) {
        geometry_msgs::msg::PoseStamped p;
        p.header.frame_id = "map";
        p.header.stamp = now();
        p.pose.position.x = cmd_res.goal_to_publish.x;
        p.pose.position.y = cmd_res.goal_to_publish.y;
        p.pose.orientation.w = 1.0;
        goal_pub_->publish(p);
        publishActiveTarget();
      }
      
      publishNavEnabled(algo_.isNavEnabled());
      publishNavMode(algo_.getNavMode());
      publishStatus();
    }
  }

  void onSetTarget(
    const msgs::srv::SetTarget::Request::SharedPtr req,
    msgs::srv::SetTarget::Response::SharedPtr res)
  {
    mission_executive::TargetEntry entry;
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

    {
      std::lock_guard<std::mutex> lk(mutex_);
      auto cmd_res = algo_.setTarget(entry);
      res->success = cmd_res.success;
      res->message = cmd_res.message;
    }
  }

  std::mutex mutex_;
  mission_executive::MissionExecutiveAlgo algo_;
  
  std::shared_ptr<GoalHandle> queued_goal_handle_;
  std::shared_ptr<GoalHandle> active_goal_handle_;

  rclcpp::CallbackGroup::SharedPtr reentrant_group_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr             nav_enabled_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           nav_mode_pub_;
  rclcpp::Publisher<msgs::msg::ActiveTarget>::SharedPtr         active_target_pub_;
  rclcpp::Publisher<msgs::msg::NavStatus>::SharedPtr            nav_status_pub_;

  rclcpp_action::Server<NavAction>::SharedPtr action_server_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr  abort_srv_;
  rclcpp::Service<msgs::srv::SetTarget>::SharedPtr     set_target_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr   teleop_srv_;

  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr           imu_sub_;
  rclcpp::Subscription<msgs::msg::PlannerEvent>::SharedPtr         planner_event_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr             global_path_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2D>::SharedPtr   aruco_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr             yolo_sub_;

  rclcpp::Client<msgs::srv::LatLonToENU>::SharedPtr latlon_client_;

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
