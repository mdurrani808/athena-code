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
//
// Inputs:
//   /global_path              (nav_msgs/Path, transient_local) — from global_planner
//   /nav_enabled              (std_msgs/Bool, reliable)         — from mission_executive
//   /obstacle_avoidance_enabled (std_msgs/Bool, reliable)       — runtime avoidance toggle
//   /scan                     (sensor_msgs/LaserScan)           — from pointcloud_to_laserscan
//   TF map→base_link                                            — current robot pose
//
// Outputs:
//   /cmd_vel          (geometry_msgs/Twist)              — linear.x + angular.z
//   ~/debug_markers   (visualization_msgs/MarkerArray)   — RViz visualization
//
// Algorithm:
//   On each 20 Hz tick, if nav_enabled and a path is loaded:
//     1. Look up current pose via TF.
//     2. Stop if within goal_tolerance_m of the last path pose.
//     3. Find the lookahead point: walk forward from the closest path pose
//        until a point is >= lookahead_dist_m from the robot.
//     4. Compute heading error from current yaw to the lookahead direction.
//     5. If obstacle_avoidance_enabled and repulsion_gain > 0:
//        - On each scan callback, transform every hit within repulsion_cutoff_m
//          into map frame and store it in obstacle_map_ with a timestamp.
//        - Each tick, evict points older than obstacle_memory_time_s.
//        - For each remaining point within repulsion_cutoff_m:
//            repulsion direction = unit vector from obstacle to robot
//            lateral component  = project onto robot's left axis (-sin(yaw), cos(yaw))
//            weight             = (cutoff - dist) / cutoff
//            lateral_sum       += lateral * weight
//        - repulsion_steering = repulsion_gain * lateral_sum
//        - Total steering = k_p_steering * heading_err + repulsion_steering
//     6. angular.z = clamped total steering, linear.x = max_speed_mps.
//
// Obstacle memory design:
//   Hits are stored in map frame, not sensor frame.  This means the repulsion
//   direction is recomputed from correct world-frame geometry every tick,
//   regardless of how the robot has turned since the hit was recorded.
//   The robot will not "forget" an obstacle just because it rotated away from it,
//   and the steering magnitude correctly reflects the actual geometric clearance.

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

struct ObstaclePoint
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
    max_speed_mps_              = get_parameter("max_speed_mps").as_double();
    max_steering_angle_rad_     = get_parameter("max_steering_angle_rad").as_double();
    lookahead_dist_m_           = get_parameter("lookahead_dist_m").as_double();
    k_p_steering_               = get_parameter("k_p_steering").as_double();
    repulsion_gain_             = get_parameter("repulsion_gain").as_double();
    repulsion_cutoff_m_         = get_parameter("repulsion_cutoff_m").as_double();
    obstacle_memory_time_s_     = get_parameter("obstacle_memory_time_s").as_double();
    obstacle_max_points_        = get_parameter("obstacle_max_points").as_int();
    obstacle_avoidance_enabled_ = get_parameter("obstacle_avoidance_enabled").as_bool();
    scan_topic_                 = get_parameter("scan_topic").as_string();
    scan_max_age_s_             = get_parameter("scan_max_age_s").as_double();
    goal_tolerance_m_           = get_parameter("goal_tolerance_m").as_double();
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

        // Log path quality — sparse paths (spacing > lookahead_dist_m) trigger the
        // findLookahead-behind-robot bug documented in the unit tests.
        double total_len = 0.0;
        double max_gap = 0.0;
        for (size_t i = 1; i < msg->poses.size(); ++i) {
          const double gap = std::hypot(
            msg->poses[i].pose.position.x - msg->poses[i-1].pose.position.x,
            msg->poses[i].pose.position.y - msg->poses[i-1].pose.position.y);
          total_len += gap;
          max_gap = std::max(max_gap, gap);
        }
        RCLCPP_INFO(get_logger(),
          "[PATH_RECV] poses=%zu total_len=%.1fm max_gap=%.2fm lookahead=%.1fm goal=(%.2f,%.2f)",
          msg->poses.size(), total_len, max_gap, lookahead_dist_m_,
          msg->poses.back().pose.position.x, msg->poses.back().pose.position.y);
        if (max_gap > lookahead_dist_m_) {
          RCLCPP_WARN(get_logger(),
            "[PATH_RECV] max_gap %.2fm > lookahead %.1fm — findLookahead will return "
            "behind-robot points once robot passes a sparse waypoint. "
            "Reduce path_resolution_m or increase lookahead_dist_m.",
            max_gap, lookahead_dist_m_);
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
        const bool was_enabled = obstacle_avoidance_enabled_;
        obstacle_avoidance_enabled_ = msg->data;
        if (was_enabled != obstacle_avoidance_enabled_) {
          RCLCPP_INFO(get_logger(), "[AVOIDANCE_%s]",
            obstacle_avoidance_enabled_ ? "ENABLED" : "DISABLED");
        }
      });

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        ++scan_msg_count_;
        // Log details on the first scan and every 100th thereafter
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
            static_cast<float>(repulsion_cutoff_m_));
        }
        latest_scan_ = msg;

        // Transform hits into map frame and add to the obstacle buffer.
        // Storing in map frame means repulsion geometry stays correct after
        // the robot turns — we don't lose obstacles that have left the FoV.
        if (!obstacle_avoidance_enabled_) return;

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
          if (static_cast<double>(r) >= repulsion_cutoff_m_) continue;
          const double alpha = msg->angle_min +
            static_cast<double>(i) * msg->angle_increment;
          const double bx = r * std::cos(alpha);
          const double by = r * std::sin(alpha);
          obstacle_map_.push_back({
            ox + bx * cos_my - by * sin_my,
            oy + bx * sin_my + by * cos_my,
            stamp});
        }

        // Cap buffer size — evict oldest entries if over limit
        if (static_cast<int>(obstacle_map_.size()) > obstacle_max_points_) {
          const size_t excess =
            obstacle_map_.size() - static_cast<size_t>(obstacle_max_points_);
          obstacle_map_.erase(obstacle_map_.begin(), obstacle_map_.begin() + excess);
        }
      });

    // 20 Hz control loop
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      [this]() { controlLoop(); });

    // ── Startup parameter summary ────────────────────────────────────────────
    RCLCPP_INFO(get_logger(),
      "VectorFieldPlanner ready  "
      "speed=%.2f steer_max=%.3f lookahead=%.1fm kp=%.2f goal_tol=%.1fm",
      max_speed_mps_, max_steering_angle_rad_, lookahead_dist_m_,
      k_p_steering_, goal_tolerance_m_);
    RCLCPP_INFO(get_logger(),
      "[AVOIDANCE CONFIG] enabled=%d scan_topic='%s' cutoff=%.2fm gain=%.3f"
      " memory_time=%.1fs max_points=%d  — publish true/false to /obstacle_avoidance_enabled to toggle",
      static_cast<int>(obstacle_avoidance_enabled_),
      scan_topic_.c_str(),
      repulsion_cutoff_m_, repulsion_gain_,
      obstacle_memory_time_s_, obstacle_max_points_);
  }

private:
  // ── Control loop (20 Hz) ───────────────────────────────────────────────────

  void controlLoop()
  {
    if (!nav_enabled_ || !path_ || path_->poses.empty()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "[NAV_IDLE] nav_enabled=%d has_path=%d",
        static_cast<int>(nav_enabled_), static_cast<int>(path_ && !path_->poses.empty()));
      return;
    }

    ++tick_count_;

    // Look up current robot pose
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

    // Yaw from quaternion (z-rotation only — planar robot)
    const auto & q   = tf.transform.rotation;
    const double yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    // Check arrival at the last pose in the path
    const auto & goal_pos = path_->poses.back().pose.position;
    const double dist_to_goal = std::hypot(goal_pos.x - rx, goal_pos.y - ry);
    if (dist_to_goal < goal_tolerance_m_) {
      RCLCPP_INFO(get_logger(),
        "[GOAL_REACHED] dist=%.3fm tol=%.3fm ticks=%u",
        dist_to_goal, goal_tolerance_m_, tick_count_);
      publishStop();
      return;
    }

    // Find lookahead point
    const size_t closest_idx = findClosestIndex(rx, ry);
    const auto [lx, ly]      = findLookahead(rx, ry, closest_idx);

    const double lookahead_dist = std::hypot(lx - rx, ly - ry);

    // Detect lookahead-behind-robot: dot(robot_forward, robot→lookahead) < 0
    const double fwd_dot = (lx - rx) * std::cos(yaw) + (ly - ry) * std::sin(yaw);
    const bool lk_behind = fwd_dot < 0.0;

    if (lk_behind) {
      RCLCPP_WARN(get_logger(),
        "[LK_BEHIND] lookahead=(%.2f,%.2f) is BEHIND robot at (%.2f,%.2f) yaw=%.1fd "
        "fwd_dot=%.3f closest_idx=%zu  — robot will steer backward. "
        "Likely cause: path spacing (%.2fm) > lookahead_dist (%.2fm).",
        lx, ly, rx, ry, yaw * 180.0 / M_PI, fwd_dot, closest_idx,
        [&]() -> double {
          if (closest_idx + 1 < path_->poses.size()) {
            return std::hypot(
              path_->poses[closest_idx+1].pose.position.x - path_->poses[closest_idx].pose.position.x,
              path_->poses[closest_idx+1].pose.position.y - path_->poses[closest_idx].pose.position.y);
          }
          return 0.0;
        }(),
        lookahead_dist_m_);
    }

    // Warn if closest_idx has not advanced for many ticks (robot stuck or looping)
    if (closest_idx == last_closest_idx_ && tick_count_ > 1) {
      ++stuck_ticks_;
      if (stuck_ticks_ == 20) {  // 1 second at 20 Hz
        RCLCPP_WARN(get_logger(),
          "[STUCK] closest_idx=%zu has not advanced for %u ticks. "
          "pos=(%.2f,%.2f) goal_dist=%.2fm. Robot may be off-path or looping.",
          closest_idx, stuck_ticks_, rx, ry, dist_to_goal);
      }
    } else {
      stuck_ticks_ = 0;
    }

    // Heading error to the lookahead
    double heading_err = std::atan2(ly - ry, lx - rx) - yaw;
    // Normalise to [-pi, pi]
    while (heading_err >  M_PI) heading_err -= 2.0 * M_PI;
    while (heading_err < -M_PI) heading_err += 2.0 * M_PI;

    // ── Obstacle avoidance (map-frame buffer) ─────────────────────────────────
    // Evict obstacle points older than obstacle_memory_time_s_.
    // This runs every tick regardless of avoidance toggle so the buffer
    // doesn't grow unboundedly when avoidance is toggled off.
    const rclcpp::Time now_time = now();
    obstacle_map_.erase(
      std::remove_if(obstacle_map_.begin(), obstacle_map_.end(),
        [&](const ObstaclePoint & p) {
          return (now_time - p.stamp).seconds() > obstacle_memory_time_s_;
        }),
      obstacle_map_.end());

    double repulsion_steering = 0.0;

    if (obstacle_avoidance_enabled_) {
      if (repulsion_gain_ <= 0.0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "[AVOIDANCE] avoidance enabled but repulsion_gain=%.3f — no effect. "
          "Set repulsion_gain > 0 in nav_params.yaml.", repulsion_gain_);
      } else if (!latest_scan_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "[AVOIDANCE] avoidance enabled but NO scan received yet on '%s'. "
          "Check pointcloud_to_laserscan is running and publishing.",
          scan_topic_.c_str());
      } else {
        const double scan_age = (now_time - latest_scan_->header.stamp).seconds();
        if (scan_age > scan_max_age_s_) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "[AVOIDANCE] scan stale: age=%.3fs > max=%.2fs — obstacle buffer may be outdated.",
            scan_age, scan_max_age_s_);
        }

        // Recompute repulsion from all buffered map-frame obstacle points.
        // For each point within repulsion_cutoff_m:
        //   unit_repulsion = (robot - obstacle) / dist   (push away from obstacle)
        //   lateral        = unit_repulsion · (-sin(yaw), cos(yaw))
        //   weight         = (cutoff - dist) / cutoff
        //   lateral_sum   += lateral * weight
        double lateral_sum   = 0.0;
        double lateral_left  = 0.0;
        double lateral_right = 0.0;
        int    active_points = 0;
        float  closest_r     = std::numeric_limits<float>::infinity();

        for (const auto & p : obstacle_map_) {
          const double dx   = rx - p.x;
          const double dy   = ry - p.y;
          const double dist = std::hypot(dx, dy);
          if (dist >= repulsion_cutoff_m_ || dist < 1e-6) continue;
          closest_r = std::min(closest_r, static_cast<float>(dist));
          const double weight  = (repulsion_cutoff_m_ - dist) / repulsion_cutoff_m_;
          // Project unit repulsion vector onto robot's left lateral axis
          const double lateral =
            (dx / dist) * (-std::sin(yaw)) + (dy / dist) * std::cos(yaw);
          lateral_sum += lateral * weight;
          ++active_points;
          if (lateral > 0.0) lateral_left  += lateral * weight;
          else               lateral_right += lateral * weight;
        }

        repulsion_steering = repulsion_gain_ * lateral_sum;

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
          "[AVOIDANCE age=%.3fs] buffered=%zu in_range=%d closest=%.2fm "
          "lateral_sum=%.3f left=%.3f right=%.3f rep_steer=%.4f",
          scan_age,
          obstacle_map_.size(), active_points,
          std::isinf(closest_r) ? -1.0f : closest_r,
          lateral_sum, lateral_left, lateral_right,
          repulsion_steering);

        if (active_points > 0) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 250,
            "[AVOIDANCE ACTIVE] %d pts within %.1fm  "
            "left=%.3f right=%.3f  rep_steer=%.4f  path_steer=%.4f",
            active_points, repulsion_cutoff_m_,
            lateral_left, lateral_right,
            repulsion_steering, k_p_steering_ * heading_err);
        }
      }
    } else {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
        "[AVOIDANCE OFF] publish 'true' to /obstacle_avoidance_enabled to activate");
    }

    // Proportional path-following steering + repulsion, clamped to physical limits
    const double steering_unclamped = k_p_steering_ * heading_err + repulsion_steering;
    const double steering = std::clamp(
      steering_unclamped,
      -max_steering_angle_rad_, max_steering_angle_rad_);

    const bool clamped = std::abs(steering_unclamped) > max_steering_angle_rad_;
    if (clamped) {
      ++consecutive_clamped_;
      if (consecutive_clamped_ == 40) {  // 2 seconds saturated
        RCLCPP_WARN(get_logger(),
          "[STEER_SAT] steering saturated for %u consecutive ticks (%.1f s). "
          "err=%.1fd repulsion=%.3f unclamped=%.3f max=%.3f. "
          "Consider increasing max_steering_angle_rad or reducing repulsion_gain.",
          consecutive_clamped_,
          consecutive_clamped_ / 20.0,
          heading_err * 180.0 / M_PI,
          repulsion_steering, steering_unclamped, max_steering_angle_rad_);
      }
    } else {
      consecutive_clamped_ = 0;
    }

    // ── Single structured log line (4 Hz) — paste directly into an LLM ──────
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 250,
      "[NAV t=%u pos=(%.2f,%.2f) yaw=%.1fd goal=%.2fm idx=%zu/%zu "
      "lk=(%.2f,%.2f) lk_dist=%.2fm fwd=%.2f err=%.1fd rep=%.3f steer=%.3f%s%s%s]",
      tick_count_, rx, ry, yaw * 180.0 / M_PI, dist_to_goal,
      closest_idx, path_->poses.size() - 1,
      lx, ly, lookahead_dist, fwd_dot,
      heading_err * 180.0 / M_PI, repulsion_steering, steering,
      clamped   ? " CLAMPED"   : "",
      lk_behind ? " LK_BEHIND" : "",
      (obstacle_avoidance_enabled_ && repulsion_gain_ > 0.0) ? " AVOID_ON" : "");

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp    = now();
    cmd.header.frame_id = base_frame_;
    cmd.twist.linear.x  = max_speed_mps_;
    cmd.twist.angular.z = steering;
    cmd_pub_->publish(cmd);

    if (publish_debug_markers_) {
      publishDebugMarkers(rx, ry, lx, ly);
    }
  }

  // ── Helpers ────────────────────────────────────────────────────────────────

  // Index of the path pose closest to (rx, ry).
  size_t findClosestIndex(double rx, double ry)
  {
    size_t best       = 0;
    double best_dist2 = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < path_->poses.size(); ++i) {
      const double dx = path_->poses[i].pose.position.x - rx;
      const double dy = path_->poses[i].pose.position.y - ry;
      const double d2 = dx * dx + dy * dy;
      if (d2 < best_dist2) {
        best_dist2 = d2;
        best = i;
      }
    }

    if (best < last_closest_idx_ && last_closest_idx_ != std::numeric_limits<size_t>::max()) {
      RCLCPP_WARN(get_logger(),
        "[closest_idx] BACKWARDS JUMP: %zu → %zu (dist=%.3f m) — "
        "robot may be closer to an earlier path segment",
        last_closest_idx_, best, std::sqrt(best_dist2));
    }
    last_closest_idx_ = best;
    return best;
  }

  // Starting from closest_idx, walk forward along the path until a pose is
  // >= lookahead_dist_m from the robot.  If the path is shorter, return its
  // last pose so the robot drives toward the goal even at close range.
  std::pair<double, double> findLookahead(
    double rx, double ry, size_t closest_idx) const
  {
    for (size_t i = closest_idx; i < path_->poses.size(); ++i) {
      const double dx = path_->poses[i].pose.position.x - rx;
      const double dy = path_->poses[i].pose.position.y - ry;
      if (std::hypot(dx, dy) >= lookahead_dist_m_) {
        return {path_->poses[i].pose.position.x,
                path_->poses[i].pose.position.y};
      }
    }
    const auto & last = path_->poses.back().pose.position;
    return {last.x, last.y};
  }

  // Zero-velocity command — always safe to call.
  void publishStop()
  {
    geometry_msgs::msg::TwistStamped stop_cmd;
    stop_cmd.header.stamp = now();
    stop_cmd.header.frame_id = base_frame_;
    cmd_pub_->publish(stop_cmd);
  }

  // Two RViz markers: a sphere at the lookahead point, and a line from the
  // robot to that point.
  void publishDebugMarkers(double rx, double ry, double lx, double ly)
  {
    visualization_msgs::msg::MarkerArray arr;
    const auto stamp = now();

    // Lookahead sphere
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

    // Line from robot to lookahead
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

  // ── Parameters ─────────────────────────────────────────────────────────────
  std::string map_frame_;
  std::string base_frame_;
  std::string cmd_vel_topic_;
  double      tf_timeout_s_;
  double      max_speed_mps_;
  double      max_steering_angle_rad_;
  double      lookahead_dist_m_;
  double      k_p_steering_;
  double      repulsion_gain_;
  double      repulsion_cutoff_m_;
  double      obstacle_memory_time_s_;
  int         obstacle_max_points_;
  bool        obstacle_avoidance_enabled_;
  std::string scan_topic_;
  double      scan_max_age_s_;
  double      goal_tolerance_m_;
  bool        publish_debug_markers_;

  // ── TF ─────────────────────────────────────────────────────────────────────
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ── ROS interfaces ──────────────────────────────────────────────────────────
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr       cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr   marker_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr                 path_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                 nav_enabled_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                 obstacle_avoidance_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr         scan_sub_;
  rclcpp::TimerBase::SharedPtr                                         timer_;

  // ── State ───────────────────────────────────────────────────────────────────
  nav_msgs::msg::Path::SharedPtr             path_;
  sensor_msgs::msg::LaserScan::SharedPtr     latest_scan_;
  bool                                       nav_enabled_{false};
  size_t                                     last_closest_idx_{std::numeric_limits<size_t>::max()};
  unsigned int                               tick_count_{0};
  unsigned int                               consecutive_clamped_{0};
  unsigned int                               stuck_ticks_{0};
  unsigned int                               scan_msg_count_{0};
  std::vector<ObstaclePoint>                 obstacle_map_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VectorFieldPlanner>());
  rclcpp::shutdown();
  return 0;
}
