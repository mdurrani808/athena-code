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

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

double VectorFieldPlannerAlgo::effectiveLookaheadDist(double current_speed) const {
  // Scale linearly from lookahead_dist_m at rest to 2x at max_speed_mps.
  // Derived entirely from existing params — no extra tuning knobs needed.
  const double t = std::clamp(std::fabs(current_speed) / params_.max_speed_mps, 0.0, 1.0);
  return params_.lookahead_dist_m * (1.0 + t);
}

double VectorFieldPlannerAlgo::approachVelocityScale(double dist_to_goal) const {
  // Ramp begins when goal enters the lookahead window (lookahead_dist_m).
  return std::clamp(dist_to_goal / params_.lookahead_dist_m, 0.0, 1.0);
}

// ---------------------------------------------------------------------------
// Interpolated lookahead carrot
// ---------------------------------------------------------------------------

VectorFieldPlannerAlgo::LookaheadResult VectorFieldPlannerAlgo::findLookahead(
    double rx, double ry, size_t closest_idx, double lookahead_dist) const
{
  if (path_.empty()) {
    return {rx, ry, false};
  }

  // Walk segments from closest_idx forward. For each segment [path_[i], path_[i+1]]
  // solve the quadratic |path_[i] + t*(path_[i+1]-path_[i]) - robot|^2 = ld^2.
  // Take the larger-t root (further along path). If t in [0,1], interpolate and return.
  const double ld2 = lookahead_dist * lookahead_dist;

  for (size_t i = closest_idx; i + 1 < path_.size(); ++i) {
    const double dx = path_[i + 1].x - path_[i].x;
    const double dy = path_[i + 1].y - path_[i].y;
    const double fx = path_[i].x - rx;
    const double fy = path_[i].y - ry;

    const double a = dx * dx + dy * dy;
    if (a < 1e-12) continue;  // degenerate segment

    const double b = 2.0 * (fx * dx + fy * dy);
    const double c = fx * fx + fy * fy - ld2;

    const double disc = b * b - 4.0 * a * c;
    if (disc < 0.0) continue;  // circle misses this segment

    // Prefer the forward intersection (larger t)
    const double t = (-b + std::sqrt(disc)) / (2.0 * a);
    if (t >= 0.0 && t <= 1.0) {
      return {
        path_[i].x + t * dx,
        path_[i].y + t * dy,
        true
      };
    }
  }

  // No segment intersection — robot is within lookahead_dist of the last point,
  // or path spacing is coarser than lookahead. Return last point.
  return {path_.back().x, path_.back().y, false};
}

// ---------------------------------------------------------------------------
// Closest index
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// Main compute
// ---------------------------------------------------------------------------

PlannerResult VectorFieldPlannerAlgo::compute(double rx, double ry, double yaw, double current_speed)
{
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

  // Feature 2: velocity-scaled lookahead
  res.effective_lookahead_dist = effectiveLookaheadDist(current_speed);

  // Feature 3: interpolated carrot
  const auto lk = findLookahead(rx, ry, res.closest_idx, res.effective_lookahead_dist);
  res.lookahead_x = lk.x;
  res.lookahead_y = lk.y;
  res.lookahead_interpolated = lk.interpolated;

  const double fwd_dot = (lk.x - rx) * std::cos(yaw) + (lk.y - ry) * std::sin(yaw);
  res.lookahead_behind = (fwd_dot < 0.0);

  res.heading_err = std::atan2(lk.y - ry, lk.x - rx) - yaw;
  while (res.heading_err > M_PI) res.heading_err -= 2.0 * M_PI;
  while (res.heading_err < -M_PI) res.heading_err += 2.0 * M_PI;

  // Obstacle repulsion
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

  // Feature 1: approach velocity scaling
  res.approach_velocity_scale = approachVelocityScale(dist_to_goal);
  const double approach_vel = params_.min_approach_linear_velocity +
    res.approach_velocity_scale * (params_.max_speed_mps - params_.min_approach_linear_velocity);
  res.linear_vel = std::min(params_.max_speed_mps, approach_vel);

  if (std::isinf(res.closest_r)) {
    res.closest_r = -1.0;
  }

  return res;
}

}  // namespace vector_field_planner
