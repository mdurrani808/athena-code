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

#include <gtest/gtest.h>
#include <cmath>
#include <vector>

#include "vector_field_planner/vector_field_planner_algo.hpp"

using namespace vector_field_planner;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static VectorFieldPlannerAlgo makeAlgo(PlannerParams p = {})
{
  VectorFieldPlannerAlgo algo;
  algo.setParams(p);
  return algo;
}

// Straight path along +X from 0 to length_m with step_m spacing
static std::vector<Pose2D> straightPath(double length_m, double step_m = 1.0)
{
  std::vector<Pose2D> path;
  for (double x = 0.0; x <= length_m + 1e-9; x += step_m) {
    path.push_back({x, 0.0});
  }
  return path;
}

// ---------------------------------------------------------------------------
// Feature 1: Approach velocity scaling
// Scaling window == lookahead_dist_m (no separate param)
// ---------------------------------------------------------------------------

TEST(ApproachVelocityScaling, FullSpeedFarFromGoal)
{
  PlannerParams p;
  p.max_speed_mps = 2.0;
  p.lookahead_dist_m = 3.0;
  p.min_approach_linear_velocity = 0.5;
  p.goal_tolerance_m = 0.5;

  auto algo = makeAlgo(p);
  algo.setPath(straightPath(20.0));  // goal at x=20, robot at origin

  // dist_to_goal=20 >> lookahead_dist_m=3 → scale clamped to 1.0
  auto res = algo.compute(0.0, 0.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(res.approach_velocity_scale, 1.0);
  EXPECT_NEAR(res.linear_vel, p.max_speed_mps, 1e-9);
}

TEST(ApproachVelocityScaling, ReducedSpeedInsideWindow)
{
  PlannerParams p;
  p.max_speed_mps = 2.0;
  p.lookahead_dist_m = 4.0;  // scaling window == 4 m
  p.min_approach_linear_velocity = 0.5;
  p.goal_tolerance_m = 0.2;

  auto algo = makeAlgo(p);
  // goal at x=2, robot at x=0 → dist_to_goal=2, lookahead_dist_m=4 → scale=0.5
  algo.setPath({{0.0, 0.0}, {2.0, 0.0}, {2.0, 0.0}});

  auto res = algo.compute(0.0, 0.0, 0.0, 0.0);
  EXPECT_NEAR(res.approach_velocity_scale, 0.5, 1e-6);
  // vel = min + scale*(max-min) = 0.5 + 0.5*1.5 = 1.25
  EXPECT_NEAR(res.linear_vel, 1.25, 1e-6);
}

TEST(ApproachVelocityScaling, NeverBelowMinSpeed)
{
  PlannerParams p;
  p.max_speed_mps = 2.0;
  p.lookahead_dist_m = 10.0;  // large window → robot well inside it
  p.min_approach_linear_velocity = 0.5;
  p.goal_tolerance_m = 0.05;

  auto algo = makeAlgo(p);
  // goal at x=0.3 — very close, scale → 0.03, floor must hold
  algo.setPath({{0.0, 0.0}, {0.2, 0.0}, {0.3, 0.0}});

  auto res = algo.compute(0.0, 0.0, 0.0, 0.0);
  EXPECT_GE(res.linear_vel, p.min_approach_linear_velocity - 1e-9);
}

// ---------------------------------------------------------------------------
// Feature 2: Velocity-scaled lookahead
// Always on: lookahead = lookahead_dist_m * (1 + speed/max_speed_mps)
// ---------------------------------------------------------------------------

TEST(VelocityScaledLookahead, ZeroSpeedUsesBaseDist)
{
  PlannerParams p;
  p.lookahead_dist_m = 3.0;
  p.max_speed_mps = 1.5;
  p.goal_tolerance_m = 0.5;

  auto algo = makeAlgo(p);
  algo.setPath(straightPath(20.0));

  // speed=0 → scale=0 → lookahead = 3.0 * 1.0 = 3.0
  auto res = algo.compute(0.0, 0.0, 0.0, 0.0);
  EXPECT_NEAR(res.effective_lookahead_dist, 3.0, 1e-9);
}

TEST(VelocityScaledLookahead, MaxSpeedDoublesDist)
{
  PlannerParams p;
  p.lookahead_dist_m = 3.0;
  p.max_speed_mps = 1.5;
  p.goal_tolerance_m = 0.5;

  auto algo = makeAlgo(p);
  algo.setPath(straightPath(20.0));

  // speed=max → scale=1 → lookahead = 3.0 * 2.0 = 6.0
  auto res = algo.compute(0.0, 0.0, 0.0, p.max_speed_mps);
  EXPECT_NEAR(res.effective_lookahead_dist, 6.0, 1e-9);
}

TEST(VelocityScaledLookahead, HalfSpeedIsIntermediate)
{
  PlannerParams p;
  p.lookahead_dist_m = 4.0;
  p.max_speed_mps = 2.0;
  p.goal_tolerance_m = 0.5;

  auto algo = makeAlgo(p);
  algo.setPath(straightPath(20.0));

  // speed=1.0 (half of max) → lookahead = 4.0 * 1.5 = 6.0
  auto res = algo.compute(0.0, 0.0, 0.0, 1.0);
  EXPECT_NEAR(res.effective_lookahead_dist, 6.0, 1e-9);
}

TEST(VelocityScaledLookahead, ClampedAboveMaxSpeed)
{
  PlannerParams p;
  p.lookahead_dist_m = 3.0;
  p.max_speed_mps = 1.5;
  p.goal_tolerance_m = 0.5;

  auto algo = makeAlgo(p);
  algo.setPath(straightPath(20.0));

  // speed beyond max → clamped to 2x
  auto res = algo.compute(0.0, 0.0, 0.0, 10.0);
  EXPECT_NEAR(res.effective_lookahead_dist, 6.0, 1e-9);
}

// ---------------------------------------------------------------------------
// Feature 3: Interpolated carrot
// ---------------------------------------------------------------------------

TEST(InterpolatedCarrot, CoarsePathInterpolated)
{
  PlannerParams p;
  p.lookahead_dist_m = 2.5;
  p.max_speed_mps = 1.5;
  p.goal_tolerance_m = 0.5;

  auto algo = makeAlgo(p);
  // Waypoints 5 m apart — carrot must land between them
  algo.setPath({{0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}});

  auto res = algo.compute(0.0, 0.0, 0.0, 0.0);
  EXPECT_TRUE(res.lookahead_interpolated);
  // Carrot is at exactly effective_lookahead_dist from robot
  const double actual_dist = std::hypot(res.lookahead_x, res.lookahead_y);
  EXPECT_NEAR(actual_dist, res.effective_lookahead_dist, 1e-6);
}

TEST(InterpolatedCarrot, NearGoalFallsBackToLastPoint)
{
  PlannerParams p;
  p.lookahead_dist_m = 5.0;
  p.max_speed_mps = 1.5;
  p.goal_tolerance_m = 0.2;

  auto algo = makeAlgo(p);
  algo.setPath({{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}});

  // Robot at x=1, only 1 m to goal, lookahead > remaining path → snap to last
  auto res = algo.compute(1.0, 0.0, 0.0, 0.0);
  EXPECT_FALSE(res.lookahead_interpolated);
  EXPECT_NEAR(res.lookahead_x, 2.0, 1e-6);
  EXPECT_NEAR(res.lookahead_y, 0.0, 1e-6);
}

TEST(InterpolatedCarrot, OffAxisPathInterpolation)
{
  PlannerParams p;
  p.lookahead_dist_m = 2.0;
  p.max_speed_mps = 1.5;
  p.goal_tolerance_m = 0.5;

  auto algo = makeAlgo(p);
  const double s = 1.0 / std::sqrt(2.0);
  algo.setPath({{0.0, 0.0}, {5.0 * s, 5.0 * s}, {10.0 * s, 10.0 * s}});

  auto res = algo.compute(0.0, 0.0, 0.0, 0.0);
  EXPECT_TRUE(res.lookahead_interpolated);
  const double actual_dist = std::hypot(res.lookahead_x, res.lookahead_y);
  EXPECT_NEAR(actual_dist, res.effective_lookahead_dist, 1e-6);
}

// ---------------------------------------------------------------------------
// Regression: existing behaviour preserved
// ---------------------------------------------------------------------------

TEST(Regression, GoalReachedWithinTolerance)
{
  PlannerParams p;
  p.goal_tolerance_m = 1.0;

  auto algo = makeAlgo(p);
  algo.setPath({{0.0, 0.0}, {1.0, 0.0}});

  auto res = algo.compute(1.0, 0.0, 0.0, 0.0);
  EXPECT_TRUE(res.goal_reached);
  EXPECT_DOUBLE_EQ(res.linear_vel, 0.0);
  EXPECT_DOUBLE_EQ(res.angular_vel, 0.0);
}

TEST(Regression, EmptyPathReturnsZero)
{
  auto algo = makeAlgo();
  auto res = algo.compute(0.0, 0.0, 0.0, 0.0);
  EXPECT_FALSE(res.goal_reached);
  EXPECT_DOUBLE_EQ(res.linear_vel, 0.0);
  EXPECT_DOUBLE_EQ(res.angular_vel, 0.0);
}

TEST(Regression, StraightAheadZeroSteering)
{
  PlannerParams p;
  p.lookahead_dist_m = 3.0;
  p.k_p_steering = 1.5;
  p.goal_tolerance_m = 0.5;
  p.max_speed_mps = 1.5;

  auto algo = makeAlgo(p);
  algo.setPath(straightPath(20.0));  // goal at x=20, well outside approach window

  // Robot facing +X along path — heading error and steering should be ~0
  auto res = algo.compute(0.0, 0.0, 0.0, 0.0);
  EXPECT_NEAR(res.heading_err, 0.0, 1e-6);
  EXPECT_NEAR(res.angular_vel, 0.0, 1e-6);
  EXPECT_NEAR(res.linear_vel, p.max_speed_mps, 1e-6);
}

// ---------------------------------------------------------------------------
// VFH* Obstacle Avoidance
// ---------------------------------------------------------------------------

TEST(VfhStar, AvoidsWallInFront)
{
  PlannerParams p;
  p.obstacle_avoidance_enabled = true;
  p.vfh_threshold = 0.5;
  p.repulsion_cutoff_m = 3.0;
  p.lookahead_dist_m = 5.0;
  p.max_speed_mps = 1.0;
  p.k_p_steering = 1.0;

  auto algo = makeAlgo(p);
  algo.setPath(straightPath(20.0));

  // Place a dense wall at x=2, from y=-2 to y=2
  std::vector<ObstaclePoint> obs;
  for (double y = -2.0; y <= 2.0; y += 0.1) {
    obs.push_back({2.0, y});
  }
  algo.updateObstacles(obs);

  // Robot at origin facing +X. Target is at x=5 (lookahead).
  // Wall is in between. VFH* should steer away.
  auto res = algo.compute(0.0, 0.0, 0.0, 0.0);

  EXPECT_GT(res.active_points, 0);
  EXPECT_NE(res.vfh_k_best, 0); // 0 corresponds to +X (roughly)
  EXPECT_GT(std::abs(res.angular_vel), 0.1);
}

TEST(VfhStar, TargetHeadingWhenClear)
{
  PlannerParams p;
  p.obstacle_avoidance_enabled = true;
  p.vfh_threshold = 0.5;
  p.repulsion_cutoff_m = 3.0;

  auto algo = makeAlgo(p);
  algo.setPath(straightPath(20.0));

  // No obstacles
  algo.updateObstacles({});

  auto res = algo.compute(0.0, 0.0, 0.0, 0.0);
  EXPECT_NEAR(res.heading_err, 0.0, 0.1);
  EXPECT_EQ(res.active_points, 0);
}

TEST(VfhStar, ThresholdFilter)
{
  PlannerParams p;
  p.obstacle_avoidance_enabled = true;
  p.vfh_threshold = 10.0; // Extremely high threshold - everything should be clear
  p.repulsion_cutoff_m = 3.0;

  auto algo = makeAlgo(p);
  algo.setPath(straightPath(20.0));

  // Place a single obstacle at x=1
  algo.updateObstacles({{1.0, 0.0}});

  auto res = algo.compute(0.0, 0.0, 0.0, 0.0);
  // High threshold means it should ignore the obstacle
  EXPECT_NEAR(res.heading_err, 0.0, 0.1);
}

TEST(VfhStar, TrapScenario)
{
  PlannerParams p;
  p.obstacle_avoidance_enabled = true;
  p.vfh_threshold = 0.5;
  p.repulsion_cutoff_m = 4.0;
  p.lookahead_dist_m = 3.0;
  p.max_speed_mps = 1.0;
  p.k_p_steering = 1.0;
  p.max_steering_angle_rad = 1.0; // Allow sharp turns

  auto algo = makeAlgo(p);
  algo.setPath(straightPath(20.0));

  // Create a U-shaped trap blocking +X
  // Front wall at x=2
  std::vector<ObstaclePoint> obs;
  for (double y = -1.5; y <= 1.5; y += 0.1) {
    obs.push_back({2.0, y});
  }
  // Side walls
  for (double x = 2.0; x <= 5.0; x += 0.2) {
    obs.push_back({x, 1.5});
    obs.push_back({x, -1.5});
  }
  algo.updateObstacles(obs);

  // Robot at origin facing +X. 
  // A local planner (VFH+) might steer slightly but stay in the U.
  // VFH* should see that all paths through the front wall are blocked
  // and steer much more aggressively to the side to go around the trap.
  auto res = algo.compute(0.0, 0.0, 0.0, 0.0);

  EXPECT_GT(res.active_points, 0);
  // Heading error should be significant (steering away from the U)
  // 0 rad is straight into the trap.
  EXPECT_GT(std::abs(res.heading_err), 0.5);
}
