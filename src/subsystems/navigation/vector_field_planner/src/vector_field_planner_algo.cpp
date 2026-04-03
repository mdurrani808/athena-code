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

#include "vector_field_planner/vector_field_planner_algo.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace vector_field_planner {

PlannerResult VectorFieldPlannerAlgo::compute(double rx, double ry, double yaw) {
  PlannerResult res;
  res.closest_r = std::numeric_limits<double>::infinity();

  if (path_.empty()) {
    return res;
  }

  const auto& goal_pos = path_.back();
  const double dist_to_goal = std::hypot(goal_pos.x - rx, goal_pos.y - ry);
  if (dist_to_goal < params_.goal_tolerance_m) {
    res.goal_reached = true;
    res.linear_vel = 0.0;
    res.angular_vel = 0.0;
    return res;
  }

  res.closest_idx = findClosestIndex(rx, ry);
  const auto [lx, ly] = findLookahead(rx, ry, res.closest_idx);
  res.lookahead_x = lx;
  res.lookahead_y = ly;

  const double fwd_dot = (lx - rx) * std::cos(yaw) + (ly - ry) * std::sin(yaw);
  res.lookahead_behind = (fwd_dot < 0.0);

  res.heading_err = std::atan2(ly - ry, lx - rx) - yaw;
  while (res.heading_err > M_PI) res.heading_err -= 2.0 * M_PI;
  while (res.heading_err < -M_PI) res.heading_err += 2.0 * M_PI;

  if (params_.obstacle_avoidance_enabled && params_.repulsion_gain > 0.0) {
    for (const auto& p : obstacles_) {
      const double dx = rx - p.x;
      const double dy = ry - p.y;
      const double dist = std::hypot(dx, dy);
      
      if (dist >= params_.repulsion_cutoff_m || dist < 1e-6) continue;
      
      res.closest_r = std::min(res.closest_r, dist);
      
      const double weight = (params_.repulsion_cutoff_m - dist) / params_.repulsion_cutoff_m;
      // Project unit repulsion vector onto robot's left lateral axis
      const double lateral = (dx / dist) * (-std::sin(yaw)) + (dy / dist) * std::cos(yaw);
      res.lateral_sum += lateral * weight;
      
      ++res.active_points;
      
      if (lateral > 0.0) {
        res.lateral_left += lateral * weight;
      } else {
        res.lateral_right += lateral * weight;
      }
    }
    res.repulsion_steering = params_.repulsion_gain * res.lateral_sum;
  }

  res.steering_unclamped = params_.k_p_steering * res.heading_err + res.repulsion_steering;
  res.angular_vel = std::clamp(res.steering_unclamped,
                               -params_.max_steering_angle_rad,
                               params_.max_steering_angle_rad);
  res.clamped = std::abs(res.steering_unclamped) > params_.max_steering_angle_rad;
  res.linear_vel = params_.max_speed_mps;

  if (std::isinf(res.closest_r)) {
    res.closest_r = -1.0;
  }

  return res;
}

size_t VectorFieldPlannerAlgo::findClosestIndex(double rx, double ry) const {
  size_t best = 0;
  double best_dist2 = std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < path_.size(); ++i) {
    const double dx = path_[i].x - rx;
    const double dy = path_[i].y - ry;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_dist2) {
      best_dist2 = d2;
      best = i;
    }
  }

  return best;
}

std::pair<double, double> VectorFieldPlannerAlgo::findLookahead(
    double rx, double ry, size_t closest_idx) const {
  if (path_.empty()) {
    return {rx, ry};
  }

  for (size_t i = closest_idx; i < path_.size(); ++i) {
    const double dx = path_[i].x - rx;
    const double dy = path_[i].y - ry;
    if (std::hypot(dx, dy) >= params_.lookahead_dist_m) {
      // Fix for "lookahead behind robot" bug:
      // When the path spacing is larger than lookahead_dist_m, the closest point
      // may be behind the robot (e.g. pose 0 that it already passed).
      // We check if this point `i` is behind the robot relative to the path segment `[i, i+1]`.
      // If it is, and we have a next point, we skip it and look ahead.
      if (i + 1 < path_.size()) {
        const double path_dx = path_[i+1].x - path_[i].x;
        const double path_dy = path_[i+1].y - path_[i].y;
        
        // Vector from robot to point `i`
        const double to_pt_dx = path_[i].x - rx;
        const double to_pt_dy = path_[i].y - ry;
        
        // Dot product of (robot -> point i) and (path direction at point i)
        const double dot = to_pt_dx * path_dx + to_pt_dy * path_dy;
        
        // If dot < 0, the point is behind the robot along the path direction.
        // We only do this check for `i == closest_idx` to prevent skipping points
        // that naturally turn backwards, but we definitely want to skip the closest point
        // if we just passed it.
        if (dot < 0.0 && i == closest_idx) {
          continue; // Skip this point, check the next one
        }
      }
      
      return {path_[i].x, path_[i].y};
    }
  }
  
  // If no point is far enough ahead (e.g. nearing goal), return the goal.
  const auto& last = path_.back();
  return {last.x, last.y};
}

}  // namespace vector_field_planner