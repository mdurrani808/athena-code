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

// global_planner_node.cpp
//
// Converts a /goal_pose into a /global_path.
//
// Phase 1 (use_costmap: false):
//   Straight-line interpolation from current TF position to goal.
//   Runs immediately on every new goal with no external dependencies.
//
// Phase 2 (use_costmap: true, costmap received):
//   A* on the OccupancyGrid from dem_costmap_converter.
//   Cost to enter a cell: dist_to_neighbor + slope_weight * (grid_value / 254).
//   Cells with grid_value >= 254 are lethal and impassable.
//   Falls back to straight-line if A* cannot find a path.

#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"

#include "global_planner/global_planner_algo.hpp"

class GlobalPlanner : public rclcpp::Node
{
public:
  GlobalPlanner() : Node("global_planner")
  {
    declare_parameter("path_resolution_m", 1.0);
    declare_parameter("use_costmap",        false);
    declare_parameter("slope_weight",       10.0);

    global_planner::PlannerParams p;
    p.path_resolution_m = get_parameter("path_resolution_m").as_double();
    p.use_costmap       = get_parameter("use_costmap").as_bool();
    p.slope_weight      = get_parameter("slope_weight").as_double();
    
    algo_.setParams(p);

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/global_path", rclcpp::QoS(1).transient_local());

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { onGoal(msg); });

    // Costmap is optional — published by dem_costmap_converter with transient_local QoS.
    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).transient_local(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        global_planner::Costmap cmap;
        cmap.width = msg->info.width;
        cmap.height = msg->info.height;
        cmap.resolution = msg->info.resolution;
        cmap.origin_x = msg->info.origin.position.x;
        cmap.origin_y = msg->info.origin.position.y;
        cmap.data = msg->data;
        algo_.setCostmap(cmap);
      });

    RCLCPP_INFO(get_logger(), "GlobalPlanner ready (use_costmap=%s)",
      p.use_costmap ? "true" : "false");
  }

private:
  // ── Goal callback ──────────────────────────────────────────────────────────

  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr goal)
  {
    // Look up current robot position in map frame
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "TF map→base_link unavailable: %s — cannot plan", ex.what());
      return;
    }

    const double sx = tf.transform.translation.x;
    const double sy = tf.transform.translation.y;
    const double gx = goal->pose.position.x;
    const double gy = goal->pose.position.y;

    RCLCPP_INFO(get_logger(), "Planning (%.2f,%.2f) → (%.2f,%.2f)", sx, sy, gx, gy);

    const auto& p = algo_.getParams();

    // Phase 2: A* if enabled and costmap is available
    if (p.use_costmap && algo_.hasCostmap()) {
      auto maybe_algo_path = algo_.planAstar(sx, sy, gx, gy);
      if (maybe_algo_path.has_value()) {
        auto msg = toPathMsg(maybe_algo_path.value());
        path_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "A* path published (%zu poses)", msg.poses.size());
        return;
      }
      RCLCPP_WARN(get_logger(), "A* failed — falling back to straight-line");
    }

    // Phase 1: straight-line
    auto algo_path = algo_.planStraightLine(sx, sy, gx, gy);
    auto msg = toPathMsg(algo_path);
    path_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Straight-line path published (%zu poses)", msg.poses.size());
  }

  nav_msgs::msg::Path toPathMsg(const std::vector<global_planner::Pose2D>& algo_path) const {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp    = now();
    path.poses.reserve(algo_path.size());

    for (const auto& pt : algo_path) {
      geometry_msgs::msg::PoseStamped p;
      p.header = path.header;
      p.pose.position.x = pt.x;
      p.pose.position.y = pt.y;
      
      const double half = pt.yaw * 0.5;
      p.pose.orientation.z = std::sin(half);
      p.pose.orientation.w = std::cos(half);
      
      path.poses.push_back(p);
    }
    
    return path;
  }

  // ── TF ─────────────────────────────────────────────────────────────────────
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ── ROS interfaces ──────────────────────────────────────────────────────────
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr              path_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr  costmap_sub_;

  // ── State ───────────────────────────────────────────────────────────────────
  global_planner::GlobalPlannerAlgo algo_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalPlanner>());
  rclcpp::shutdown();
  return 0;
}
