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

#pragma once

#include <vector>
#include <cstddef>
#include <utility>

namespace vector_field_planner {

struct Pose2D {
  double x;
  double y;
};

struct ObstaclePoint {
  double x;
  double y;
  // We can add a timestamp if needed, but eviction is currently handled
  // by the ROS node before passing to the algorithm.
};

struct PlannerParams {
  double lookahead_dist_m = 3.0;
  double k_p_steering = 1.5;
  double max_steering_angle_rad = 0.5;
  double max_speed_mps = 1.5;
  double goal_tolerance_m = 1.5;

  bool obstacle_avoidance_enabled = false;
  double repulsion_gain = 0.5;
  double repulsion_cutoff_m = 3.0;
};

struct PlannerResult {
  double linear_vel = 0.0;
  double angular_vel = 0.0;
  bool goal_reached = false;
  double lookahead_x = 0.0;
  double lookahead_y = 0.0;
  size_t closest_idx = 0;
  double heading_err = 0.0;
  double repulsion_steering = 0.0;
  double steering_unclamped = 0.0;
  bool clamped = false;
  bool lookahead_behind = false;

  // Debug info for obstacle avoidance
  double lateral_sum = 0.0;
  double lateral_left = 0.0;
  double lateral_right = 0.0;
  int active_points = 0;
  double closest_r = -1.0;
};

class VectorFieldPlannerAlgo {
public:
  VectorFieldPlannerAlgo() = default;

  void setParams(const PlannerParams& params) {
    params_ = params;
  }

  const PlannerParams& getParams() const {
    return params_;
  }

  void setPath(const std::vector<Pose2D>& path) {
    path_ = path;
  }

  // obstacle_map should only contain valid (non-stale) points in map frame
  void updateObstacles(const std::vector<ObstaclePoint>& obstacles) {
    obstacles_ = obstacles;
  }

  PlannerResult compute(double rx, double ry, double yaw);

  // Expose for testing and debugging
  size_t findClosestIndex(double rx, double ry) const;
  std::pair<double, double> findLookahead(double rx, double ry, size_t closest_idx) const;

private:
  PlannerParams params_;
  std::vector<Pose2D> path_;
  std::vector<ObstaclePoint> obstacles_;
};

}  // namespace vector_field_planner
