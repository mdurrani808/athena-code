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
#include <queue>
#include <string>
#include <tuple>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"

class GlobalPlanner : public rclcpp::Node
{
public:
  GlobalPlanner() : Node("global_planner")
  {
    declare_parameter("path_resolution_m", 1.0);
    declare_parameter("use_costmap",        false);
    declare_parameter("slope_weight",       10.0);

    path_resolution_m_ = get_parameter("path_resolution_m").as_double();
    use_costmap_       = get_parameter("use_costmap").as_bool();
    slope_weight_      = get_parameter("slope_weight").as_double();

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
        costmap_ = *msg;
      });

    RCLCPP_INFO(get_logger(), "GlobalPlanner ready (use_costmap=%s)",
      use_costmap_ ? "true" : "false");
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

    // Phase 2: A* if enabled and costmap is available
    if (use_costmap_ && costmap_.has_value()) {
      auto maybe_path = planAstar(sx, sy, gx, gy);
      if (maybe_path.has_value()) {
        path_pub_->publish(*maybe_path);
        RCLCPP_INFO(get_logger(), "A* path published (%zu poses)", maybe_path->poses.size());
        return;
      }
      RCLCPP_WARN(get_logger(), "A* failed — falling back to straight-line");
    }

    // Phase 1: straight-line
    auto path = planStraightLine(sx, sy, gx, gy);
    path_pub_->publish(path);
    RCLCPP_INFO(get_logger(), "Straight-line path published (%zu poses)", path.poses.size());
  }

  // ── Phase 1: straight-line interpolation ──────────────────────────────────

  nav_msgs::msg::Path planStraightLine(
    double sx, double sy, double gx, double gy) const
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp    = now();

    const double dist = std::hypot(gx - sx, gy - sy);

    if (dist < 1e-6) {
      // Already at goal — single-pose path
      geometry_msgs::msg::PoseStamped p;
      p.header              = path.header;
      p.pose.position.x     = gx;
      p.pose.position.y     = gy;
      p.pose.orientation.w  = 1.0;
      path.poses.push_back(p);
      return path;
    }

    // Number of samples including both endpoints
    const int n = std::max(2, static_cast<int>(std::ceil(dist / path_resolution_m_)) + 1);

    // Heading quaternion: constant along the line
    const double yaw  = std::atan2(gy - sy, gx - sx);
    const double half = yaw * 0.5;
    const double qz   = std::sin(half);
    const double qw   = std::cos(half);

    path.poses.reserve(n);
    for (int i = 0; i < n; ++i) {
      const double t = static_cast<double>(i) / (n - 1);
      geometry_msgs::msg::PoseStamped p;
      p.header              = path.header;
      p.pose.position.x     = sx + t * (gx - sx);
      p.pose.position.y     = sy + t * (gy - sy);
      p.pose.orientation.z  = qz;
      p.pose.orientation.w  = qw;
      path.poses.push_back(p);
    }

    return path;
  }

  // ── Phase 2: A* on OccupancyGrid ──────────────────────────────────────────
  //
  // g(cell) = dist_to_neighbor + slope_weight_ * (grid_value / 254)
  //
  // OccupancyGrid.data is int8[], but dem_costmap_converter writes values in
  // the nav2 range 0–254.  Cast to uint8_t before comparing to avoid sign bugs
  // (254 as int8_t = -2, which would incorrectly pass a < 254 guard).

  std::optional<nav_msgs::msg::Path> planAstar(
    double sx, double sy, double gx, double gy) const
  {
    const auto & info = costmap_->info;
    const int    W   = static_cast<int>(info.width);
    const int    H   = static_cast<int>(info.height);
    const double res = info.resolution;
    const double ox  = info.origin.position.x;
    const double oy  = info.origin.position.y;

    // World → grid cell (col, row)
    auto toGrid = [&](double wx, double wy, int & col, int & row) -> bool {
      col = static_cast<int>((wx - ox) / res);
      row = static_cast<int>((wy - oy) / res);
      return col >= 0 && col < W && row >= 0 && row < H;
    };

    // Grid cell center → world
    auto toWorld = [&](int col, int row, double & wx, double & wy) {
      wx = ox + (col + 0.5) * res;
      wy = oy + (row + 0.5) * res;
    };

    int sc, sr, gc, gr;
    if (!toGrid(sx, sy, sc, sr)) {
      RCLCPP_WARN(get_logger(), "Start (%.2f,%.2f) is outside costmap bounds", sx, sy);
      return std::nullopt;
    }
    if (!toGrid(gx, gy, gc, gr)) {
      RCLCPP_WARN(get_logger(), "Goal (%.2f,%.2f) is outside costmap bounds", gx, gy);
      return std::nullopt;
    }

    // Reject lethal goal cell
    const auto goal_val = static_cast<uint8_t>(costmap_->data[gr * W + gc]);
    if (goal_val >= 254) {
      RCLCPP_WARN(get_logger(), "Goal cell is lethal (cost=%u) — A* cannot plan", goal_val);
      return std::nullopt;
    }

    // A* with 8-connected grid
    // Open set: (f_cost, col, row)
    using State = std::tuple<double, int, int>;
    std::priority_queue<State, std::vector<State>, std::greater<State>> open;

    std::vector<double> g_cost(W * H, std::numeric_limits<double>::infinity());
    std::vector<int>    parent(W * H, -1);

    const int s_idx = sr * W + sc;
    g_cost[s_idx] = 0.0;
    open.emplace(0.0, sc, sr);

    // 8-directional moves: (dcol, drow, euclidean_distance)
    constexpr int    dcol[8] = { 1, -1,  0,  0,  1,  1, -1, -1};
    constexpr int    drow[8] = { 0,  0,  1, -1,  1, -1,  1, -1};
    constexpr double step[8] = { 1,  1,  1,  1,  M_SQRT2, M_SQRT2, M_SQRT2, M_SQRT2};

    bool found = false;
    while (!open.empty()) {
      auto [f, cx, cy] = open.top(); open.pop();

      const int idx = cy * W + cx;
      if (f > g_cost[idx] + 1e-9) continue;   // stale entry
      if (cx == gc && cy == gr) { found = true; break; }

      for (int d = 0; d < 8; ++d) {
        const int nx = cx + dcol[d];
        const int ny = cy + drow[d];
        if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;

        const auto cell_val = static_cast<uint8_t>(costmap_->data[ny * W + nx]);
        if (cell_val >= 254) continue;   // lethal — cannot enter

        const double move_cost =
          step[d] * res + slope_weight_ * static_cast<double>(cell_val) / 254.0;

        const double ng = g_cost[idx] + move_cost;
        const int    n_idx = ny * W + nx;
        if (ng < g_cost[n_idx]) {
          g_cost[n_idx] = ng;
          parent[n_idx] = idx;
          // Euclidean heuristic (admissible)
          const double h = std::hypot(nx - gc, ny - gr) * res;
          open.emplace(ng + h, nx, ny);
        }
      }
    }

    if (!found) {
      RCLCPP_WARN(get_logger(), "A* exhausted open set — no path found");
      return std::nullopt;
    }

    // Trace path from goal back to start, then reverse
    std::vector<std::pair<int, int>> cells;  // (col, row)
    for (int cur = gr * W + gc; cur != -1; cur = parent[cur]) {
      cells.emplace_back(cur % W, cur / W);
    }
    std::reverse(cells.begin(), cells.end());

    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp    = now();
    path.poses.reserve(cells.size());

    for (size_t i = 0; i < cells.size(); ++i) {
      double wx, wy;
      toWorld(cells[i].first, cells[i].second, wx, wy);

      geometry_msgs::msg::PoseStamped p;
      p.header          = path.header;
      p.pose.position.x = wx;
      p.pose.position.y = wy;

      // Point toward the next cell; last pose inherits previous heading
      if (i + 1 < cells.size()) {
        double nwx, nwy;
        toWorld(cells[i + 1].first, cells[i + 1].second, nwx, nwy);
        const double yaw  = std::atan2(nwy - wy, nwx - wx);
        const double half = yaw * 0.5;
        p.pose.orientation.z = std::sin(half);
        p.pose.orientation.w = std::cos(half);
      } else if (!path.poses.empty()) {
        p.pose.orientation = path.poses.back().pose.orientation;
      } else {
        p.pose.orientation.w = 1.0;
      }

      path.poses.push_back(p);
    }

    return path;
  }

  // ── Parameters ─────────────────────────────────────────────────────────────
  double path_resolution_m_{1.0};
  bool   use_costmap_{false};
  double slope_weight_{10.0};

  // ── TF ─────────────────────────────────────────────────────────────────────
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ── ROS interfaces ──────────────────────────────────────────────────────────
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr              path_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr  costmap_sub_;

  // ── State ───────────────────────────────────────────────────────────────────
  std::optional<nav_msgs::msg::OccupancyGrid> costmap_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalPlanner>());
  rclcpp::shutdown();
  return 0;
}
