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
  double vfh_threshold = 0.5;

  // Approach velocity scaling: ramp speed down to min_approach_linear_velocity
  // as the robot closes within lookahead_dist_m of the goal. The velocity-scaled
  // lookahead and approach window are both derived from lookahead_dist_m and
  // max_speed_mps so no additional parameters are needed.
  double min_approach_linear_velocity = 0.3;
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

  // VFH* debug info
  int vfh_k_best = -1;
  int vfh_candidates_n = 0;

  // Debug info for new features
  double effective_lookahead_dist = 0.0;  // actual lookahead used this tick
  bool lookahead_interpolated = false;     // true when carrot was interpolated between waypoints
  double approach_velocity_scale = 1.0;   // fraction applied by approach scaling (1.0 = full speed)
};

class VectorFieldPlannerAlgo {
public:
  VectorFieldPlannerAlgo() = default;

  /*
   * Sets the parameters for the vector field planner, such as speeds and lookahead distance.
   * Param: params - The new planner configuration.
   */
  void setParams(const PlannerParams& params) {
    params_ = params;
  }

  /*
   * Returns the current planner parameters.
   */
  const PlannerParams& getParams() const {
    return params_;
  }

  /*
   * Sets the path that the planner should follow.
   * Param: path - Sequence of 2D poses constituting the path.
   */
  void setPath(const std::vector<Pose2D>& path) {
    path_ = path;
  }

  /*
   * Updates the local obstacle map used for repulsive avoidance.
   * Expects valid (non-stale) points in the map frame.
   * Param: obstacles - A list of obstacle points.
   */
  void updateObstacles(const std::vector<ObstaclePoint>& obstacles) {
    obstacles_ = obstacles;
  }

  /*
   * Computes the velocity and steering commands to follow the path and avoid obstacles.
   * Param: rx           - Current robot X position (map frame).
   * Param: ry           - Current robot Y position (map frame).
   * Param: yaw          - Current robot heading (yaw, radians).
   * Param: current_speed - Current linear speed (m/s), used for velocity-scaled lookahead.
   * Returns: PlannerResult containing commanded linear/angular velocities and diagnostic info.
   */
  PlannerResult compute(double rx, double ry, double yaw, double current_speed);

  // Exposed for testing and debugging
  size_t findClosestIndex(double rx, double ry) const;

  /*
   * Finds the lookahead carrot point at exactly lookahead_dist from the robot by
   * linearly interpolating between waypoints.  Falls back to the last path point
   * when no segment intersection is found (i.e. near goal).
   * Returns {x, y, interpolated} where interpolated=true when the point lies between waypoints.
   */
  struct LookaheadResult {
    double x;
    double y;
    bool interpolated;
  };
  LookaheadResult findLookahead(double rx, double ry, size_t closest_idx,
                                double lookahead_dist) const;

private:
  PlannerParams params_;
  std::vector<Pose2D> path_;
  std::vector<ObstaclePoint> obstacles_;
  int prev_k_{0};

  double effectiveLookaheadDist(double current_speed) const;
  double approachVelocityScale(double dist_to_goal) const;

  // VFH* internals
  std::vector<double> buildHistogram(const std::vector<ObstaclePoint>& obs,
                                     double rx, double ry) const;
  std::vector<double> smoothHistogram(const std::vector<double>& h) const;
  std::vector<int>    findCandidates(const std::vector<double>& h,
                                     int k_target, int k_prev) const;
  int                 runAstar(double rx, double ry, double yaw,
                               int k_target, int k_prev) const;
};

}  // namespace vector_field_planner
