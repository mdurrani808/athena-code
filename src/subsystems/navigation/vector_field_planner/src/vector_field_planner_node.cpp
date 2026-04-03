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

// vector_field_planner_node.cpp
//
// Pure-pursuit path follower for an Ackermann rover with optional obstacle avoidance.

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"

#include "vector_field_planner/vector_field_planner_algo.hpp"

struct StampedObstaclePoint
{
  double x, y;        // map frame
  rclcpp::Time stamp;
};

class VectorFieldPlanner : public rclcpp::Node
{
public:
  VectorFieldPlanner() : Node("vector_field_planner")
  {
    declare_parameter("map_frame",              std::string("map"));
    declare_parameter("base_frame",             std::string("base_link"));
    declare_parameter("cmd_vel_topic",          std::string("/front_ackermann_controller/reference"));
    declare_parameter("tf_timeout_s",           0.1);
    declare_parameter("max_speed_mps",          1.5);
    declare_parameter("max_steering_angle_rad", 0.5);
    declare_parameter("lookahead_dist_m",       3.0);
    declare_parameter("k_p_steering",           1.5);
    declare_parameter("repulsion_gain",               0.5);
    declare_parameter("repulsion_cutoff_m",           3.0);
    declare_parameter("obstacle_memory_time_s",       3.0);
    declare_parameter("obstacle_max_points",          2000);
    declare_parameter("obstacle_avoidance_enabled",   false);
    declare_parameter("scan_topic",                   std::string("/scan"));
    declare_parameter("scan_max_age_s",               0.5);
    declare_parameter("goal_tolerance_m",             1.5);
    declare_parameter("publish_debug_markers",        true);

    map_frame_                  = get_parameter("map_frame").as_string();
    base_frame_                 = get_parameter("base_frame").as_string();
    cmd_vel_topic_              = get_parameter("cmd_vel_topic").as_string();
    tf_timeout_s_               = get_parameter("tf_timeout_s").as_double();
    
    vector_field_planner::PlannerParams p;
    p.max_speed_mps              = get_parameter("max_speed_mps").as_double();
    p.max_steering_angle_rad     = get_parameter("max_steering_angle_rad").as_double();
    p.lookahead_dist_m           = get_parameter("lookahead_dist_m").as_double();
    p.k_p_steering               = get_parameter("k_p_steering").as_double();
    p.repulsion_gain             = get_parameter("repulsion_gain").as_double();
    p.repulsion_cutoff_m         = get_parameter("repulsion_cutoff_m").as_double();
    p.obstacle_avoidance_enabled = get_parameter("obstacle_avoidance_enabled").as_bool();
    p.goal_tolerance_m           = get_parameter("goal_tolerance_m").as_double();
    
    algo_.setParams(p);

    obstacle_memory_time_s_     = get_parameter("obstacle_memory_time_s").as_double();
    obstacle_max_points_        = get_parameter("obstacle_max_points").as_int();
    scan_topic_                 = get_parameter("scan_topic").as_string();
    scan_max_age_s_             = get_parameter("scan_max_age_s").as_double();
    publish_debug_markers_      = get_parameter("publish_debug_markers").as_bool();

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_topic_, 10);

    if (publish_debug_markers_) {
      marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "~/debug_markers", 10);
    }

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/global_path", rclcpp::QoS(1).transient_local(),
      [this](const nav_msgs::msg::Path::SharedPtr msg) {
        path_ = msg;
        last_closest_idx_ = std::numeric_limits<size_t>::max();
        tick_count_ = 0;
        consecutive_clamped_ = 0;
        obstacle_map_.clear();

        std::vector<vector_field_planner::Pose2D> algo_path;
        algo_path.reserve(msg->poses.size());

        double total_len = 0.0;
        double max_gap = 0.0;
        for (size_t i = 0; i < msg->poses.size(); ++i) {
          algo_path.push_back({msg->poses[i].pose.position.x, msg->poses[i].pose.position.y});
          if (i > 0) {
            const double gap = std::hypot(
              msg->poses[i].pose.position.x - msg->poses[i-1].pose.position.x,
              msg->poses[i].pose.position.y - msg->poses[i-1].pose.position.y);
            total_len += gap;
            max_gap = std::max(max_gap, gap);
          }
        }
        
        algo_.setPath(algo_path);

        const double lookahead_dist_m = algo_.getParams().lookahead_dist_m;
        RCLCPP_INFO(get_logger(),
          "[PATH_RECV] poses=%zu total_len=%.1fm max_gap=%.2fm lookahead=%.1fm goal=(%.2f,%.2f)",
          msg->poses.size(), total_len, max_gap, lookahead_dist_m,
          msg->poses.back().pose.position.x, msg->poses.back().pose.position.y);
        if (max_gap > lookahead_dist_m) {
          RCLCPP_WARN(get_logger(),
            "[PATH_RECV] max_gap %.2fm > lookahead %.1fm — consider reducing path_resolution_m "
            "or increasing lookahead_dist_m.",
            max_gap, lookahead_dist_m);
        }
      });

    nav_enabled_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/nav_enabled", rclcpp::QoS(1).reliable(),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        const bool was_enabled = nav_enabled_;
        nav_enabled_ = msg->data;
        if (was_enabled && !nav_enabled_) {
          RCLCPP_INFO(get_logger(), "[NAV_DISABLED] stopping after %u ticks", tick_count_);
          tick_count_ = 0;
          consecutive_clamped_ = 0;
          stuck_ticks_ = 0;
          obstacle_map_.clear();
          publishStop();
        } else if (!was_enabled && nav_enabled_) {
          RCLCPP_INFO(get_logger(), "[NAV_ENABLED] starting navigation");
        }
      });

    obstacle_avoidance_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/obstacle_avoidance_enabled", rclcpp::QoS(1).reliable(),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        vector_field_planner::PlannerParams p = algo_.getParams();
        const bool was_enabled = p.obstacle_avoidance_enabled;
        p.obstacle_avoidance_enabled = msg->data;
        algo_.setParams(p);
        
        if (was_enabled != p.obstacle_avoidance_enabled) {
          RCLCPP_INFO(get_logger(), "[AVOIDANCE_%s]",
            p.obstacle_avoidance_enabled ? "ENABLED" : "DISABLED");
        }
      });

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        ++scan_msg_count_;
        const auto p = algo_.getParams();
        if (scan_msg_count_ == 1 || scan_msg_count_ % 100 == 0) {
          float min_r = std::numeric_limits<float>::infinity();
          int   valid = 0;
          for (float r : msg->ranges) {
            if (r >= msg->range_min && r <= msg->range_max) {
              min_r = std::min(min_r, r);
              ++valid;
            }
          }
          RCLCPP_INFO(get_logger(),
            "[SCAN #%u] frame='%s' beams=%zu angle=[%.2f,%.2f]rad"
            " range=[%.2f,%.2f]m  valid_beams=%d min_range=%.2fm cutoff=%.2fm",
            scan_msg_count_,
            msg->header.frame_id.c_str(),
            msg->ranges.size(),
            msg->angle_min, msg->angle_max,
            msg->range_min, msg->range_max,
            valid,
            std::isinf(min_r) ? -1.0f : min_r,
            static_cast<float>(p.repulsion_cutoff_m));
        }
        latest_scan_ = msg;

        if (!p.obstacle_avoidance_enabled) return;

        geometry_msgs::msg::TransformStamped tf_to_map;
        try {
          tf_to_map = tf_buffer_->lookupTransform(
            map_frame_, msg->header.frame_id,
            tf2::TimePointZero,
            tf2::durationFromSec(tf_timeout_s_));
        } catch (const tf2::TransformException & ex) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "[SCAN_TF] %s→%s failed: %s — skipping buffer update",
            msg->header.frame_id.c_str(), map_frame_.c_str(), ex.what());
          return;
        }

        const double ox = tf_to_map.transform.translation.x;
        const double oy = tf_to_map.transform.translation.y;
        const auto & qm = tf_to_map.transform.rotation;
        const double map_yaw = std::atan2(
          2.0 * (qm.w * qm.z + qm.x * qm.y),
          1.0 - 2.0 * (qm.y * qm.y + qm.z * qm.z));
        const double cos_my = std::cos(map_yaw);
        const double sin_my = std::sin(map_yaw);

        const rclcpp::Time stamp = now();
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
          const float r = msg->ranges[i];
          if (r < msg->range_min || r > msg->range_max) continue;
          if (static_cast<double>(r) >= p.repulsion_cutoff_m) continue;
          const double alpha = msg->angle_min +
            static_cast<double>(i) * msg->angle_increment;
          const double bx = r * std::cos(alpha);
          const double by = r * std::sin(alpha);
          obstacle_map_.push_back({
            ox + bx * cos_my - by * sin_my,
            oy + bx * sin_my + by * cos_my,
            stamp});
        }

        if (static_cast<int>(obstacle_map_.size()) > obstacle_max_points_) {
          const size_t excess =
            obstacle_map_.size() - static_cast<size_t>(obstacle_max_points_);
          obstacle_map_.erase(obstacle_map_.begin(), obstacle_map_.begin() + excess);
        }
      });

    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      [this]() { controlLoop(); });

    const auto current_params = algo_.getParams();
    RCLCPP_INFO(get_logger(),
      "VectorFieldPlanner ready  "
      "speed=%.2f steer_max=%.3f lookahead=%.1fm kp=%.2f goal_tol=%.1fm",
      current_params.max_speed_mps, current_params.max_steering_angle_rad, current_params.lookahead_dist_m,
      current_params.k_p_steering, current_params.goal_tolerance_m);
    RCLCPP_INFO(get_logger(),
      "[AVOIDANCE CONFIG] enabled=%d scan_topic='%s' cutoff=%.2fm gain=%.3f"
      " memory_time=%.1fs max_points=%d  — publish true/false to /obstacle_avoidance_enabled to toggle",
      static_cast<int>(current_params.obstacle_avoidance_enabled),
      scan_topic_.c_str(),
      current_params.repulsion_cutoff_m, current_params.repulsion_gain,
      obstacle_memory_time_s_, obstacle_max_points_);
  }

private:
  void controlLoop()
  {
    if (!nav_enabled_ || !path_ || path_->poses.empty()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "[NAV_IDLE] nav_enabled=%d has_path=%d",
        static_cast<int>(nav_enabled_), static_cast<int>(path_ && !path_->poses.empty()));
      return;
    }

    ++tick_count_;

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(
        map_frame_, base_frame_,
        tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_s_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "[TF_FAIL] %s→%s: %s  (tick=%u)",
        map_frame_.c_str(), base_frame_.c_str(), ex.what(), tick_count_);
      return;
    }

    const double rx = tf.transform.translation.x;
    const double ry = tf.transform.translation.y;

    const auto & q   = tf.transform.rotation;
    const double yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    const rclcpp::Time now_time = now();
    obstacle_map_.erase(
      std::remove_if(obstacle_map_.begin(), obstacle_map_.end(),
        [&](const StampedObstaclePoint & p) {
          return (now_time - p.stamp).seconds() > obstacle_memory_time_s_;
        }),
      obstacle_map_.end());

    const auto p = algo_.getParams();
    
    if (p.obstacle_avoidance_enabled) {
      if (p.repulsion_gain <= 0.0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "[AVOIDANCE] avoidance enabled but repulsion_gain=%.3f — no effect.", p.repulsion_gain);
      } else if (!latest_scan_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "[AVOIDANCE] avoidance enabled but NO scan received yet on '%s'.", scan_topic_.c_str());
      } else {
        const double scan_age = (now_time - latest_scan_->header.stamp).seconds();
        if (scan_age > scan_max_age_s_) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "[AVOIDANCE] scan stale: age=%.3fs > max=%.2fs", scan_age, scan_max_age_s_);
        }
      }
    } else {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
        "[AVOIDANCE OFF] publish 'true' to /obstacle_avoidance_enabled to activate");
    }

    std::vector<vector_field_planner::ObstaclePoint> algo_obstacles;
    algo_obstacles.reserve(obstacle_map_.size());
    for (const auto& op : obstacle_map_) {
      algo_obstacles.push_back({op.x, op.y});
    }
    algo_.updateObstacles(algo_obstacles);

    const auto res = algo_.compute(rx, ry, yaw);

    if (res.goal_reached) {
      const auto & goal_pos = path_->poses.back().pose.position;
      const double dist_to_goal = std::hypot(goal_pos.x - rx, goal_pos.y - ry);
      RCLCPP_INFO(get_logger(),
        "[GOAL_REACHED] dist=%.3fm tol=%.3fm ticks=%u",
        dist_to_goal, p.goal_tolerance_m, tick_count_);
      publishStop();
      return;
    }

    if (res.closest_idx < last_closest_idx_ && last_closest_idx_ != std::numeric_limits<size_t>::max()) {
      RCLCPP_WARN(get_logger(),
        "[closest_idx] BACKWARDS JUMP: %zu → %zu — robot may be closer to an earlier path segment",
        last_closest_idx_, res.closest_idx);
    }
    last_closest_idx_ = res.closest_idx;

    if (res.lookahead_behind) {
      RCLCPP_WARN(get_logger(),
        "[LK_BEHIND] lookahead=(%.2f,%.2f) is BEHIND robot at (%.2f,%.2f) yaw=%.1fd closest_idx=%zu",
        res.lookahead_x, res.lookahead_y, rx, ry, yaw * 180.0 / M_PI, res.closest_idx);
    }

    const auto & goal_pos = path_->poses.back().pose.position;
    const double dist_to_goal = std::hypot(goal_pos.x - rx, goal_pos.y - ry);
    
    if (res.closest_idx == last_closest_idx_ && tick_count_ > 1) {
      ++stuck_ticks_;
      if (stuck_ticks_ == 20) {
        RCLCPP_WARN(get_logger(),
          "[STUCK] closest_idx=%zu has not advanced for %u ticks. "
          "pos=(%.2f,%.2f) goal_dist=%.2fm.", res.closest_idx, stuck_ticks_, rx, ry, dist_to_goal);
      }
    } else {
      stuck_ticks_ = 0;
    }

    if (p.obstacle_avoidance_enabled && p.repulsion_gain > 0.0 && latest_scan_) {
      const double scan_age = (now_time - latest_scan_->header.stamp).seconds();
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
        "[AVOIDANCE age=%.3fs] buffered=%zu in_range=%d closest=%.2fm "
        "lateral_sum=%.3f left=%.3f right=%.3f rep_steer=%.4f",
        scan_age, obstacle_map_.size(), res.active_points, res.closest_r,
        res.lateral_sum, res.lateral_left, res.lateral_right, res.repulsion_steering);

      if (res.active_points > 0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 250,
          "[AVOIDANCE ACTIVE] %d pts within %.1fm  left=%.3f right=%.3f  rep_steer=%.4f  path_steer=%.4f",
          res.active_points, p.repulsion_cutoff_m, res.lateral_left, res.lateral_right,
          res.repulsion_steering, p.k_p_steering * res.heading_err);
      }
    }

    if (res.clamped) {
      ++consecutive_clamped_;
      if (consecutive_clamped_ == 40) {
        RCLCPP_WARN(get_logger(),
          "[STEER_SAT] steering saturated for %u consecutive ticks (%.1f s). "
          "err=%.1fd repulsion=%.3f unclamped=%.3f max=%.3f.",
          consecutive_clamped_, consecutive_clamped_ / 20.0,
          res.heading_err * 180.0 / M_PI, res.repulsion_steering, 
          res.steering_unclamped, p.max_steering_angle_rad);
      }
    } else {
      consecutive_clamped_ = 0;
    }

    const double lookahead_dist = std::hypot(res.lookahead_x - rx, res.lookahead_y - ry);
    
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 250,
      "[NAV t=%u pos=(%.2f,%.2f) yaw=%.1fd goal=%.2fm idx=%zu/%zu "
      "lk=(%.2f,%.2f) lk_dist=%.2fm err=%.1fd rep=%.3f steer=%.3f%s%s%s]",
      tick_count_, rx, ry, yaw * 180.0 / M_PI, dist_to_goal,
      res.closest_idx, path_->poses.size() - 1,
      res.lookahead_x, res.lookahead_y, lookahead_dist,
      res.heading_err * 180.0 / M_PI, res.repulsion_steering, res.angular_vel,
      res.clamped   ? " CLAMPED"   : "",
      res.lookahead_behind ? " LK_BEHIND" : "",
      (p.obstacle_avoidance_enabled && p.repulsion_gain > 0.0) ? " AVOID_ON" : "");

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp    = now();
    cmd.header.frame_id = base_frame_;
    cmd.twist.linear.x  = res.linear_vel;
    cmd.twist.angular.z = res.angular_vel;
    cmd_pub_->publish(cmd);

    if (publish_debug_markers_) {
      publishDebugMarkers(rx, ry, res.lookahead_x, res.lookahead_y);
    }
  }

  void publishStop()
  {
    geometry_msgs::msg::TwistStamped stop_cmd;
    stop_cmd.header.stamp = now();
    stop_cmd.header.frame_id = base_frame_;
    cmd_pub_->publish(stop_cmd);
  }

  void publishDebugMarkers(double rx, double ry, double lx, double ly)
  {
    visualization_msgs::msg::MarkerArray arr;
    const auto stamp = now();

    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id = map_frame_;
    sphere.header.stamp    = stamp;
    sphere.ns              = "vector_field_planner";
    sphere.id              = 0;
    sphere.type            = visualization_msgs::msg::Marker::SPHERE;
    sphere.action          = visualization_msgs::msg::Marker::ADD;
    sphere.pose.position.x = lx;
    sphere.pose.position.y = ly;
    sphere.pose.orientation.w = 1.0;
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.5;
    sphere.color.r = 1.0f;
    sphere.color.g = 0.5f;
    sphere.color.a = 1.0f;
    arr.markers.push_back(sphere);

    visualization_msgs::msg::Marker line;
    line.header  = sphere.header;
    line.ns      = "vector_field_planner";
    line.id      = 1;
    line.type    = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action  = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.1;
    line.color.r = 1.0f;
    line.color.a = 1.0f;
    geometry_msgs::msg::Point p1, p2;
    p1.x = rx; p1.y = ry;
    p2.x = lx; p2.y = ly;
    line.points = {p1, p2};
    arr.markers.push_back(line);

    marker_pub_->publish(arr);
  }

  std::string map_frame_;
  std::string base_frame_;
  std::string cmd_vel_topic_;
  double      tf_timeout_s_;
  double      obstacle_memory_time_s_;
  int         obstacle_max_points_;
  std::string scan_topic_;
  double      scan_max_age_s_;
  bool        publish_debug_markers_;

  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr       cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr   marker_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr                 path_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                 nav_enabled_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                 obstacle_avoidance_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr         scan_sub_;
  rclcpp::TimerBase::SharedPtr                                         timer_;

  nav_msgs::msg::Path::SharedPtr             path_;
  sensor_msgs::msg::LaserScan::SharedPtr     latest_scan_;
  bool                                       nav_enabled_{false};
  size_t                                     last_closest_idx_{std::numeric_limits<size_t>::max()};
  unsigned int                               tick_count_{0};
  unsigned int                               consecutive_clamped_{0};
  unsigned int                               stuck_ticks_{0};
  unsigned int                               scan_msg_count_{0};
  std::vector<StampedObstaclePoint>          obstacle_map_;
  vector_field_planner::VectorFieldPlannerAlgo algo_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VectorFieldPlanner>());
  rclcpp::shutdown();
  return 0;
}
