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

// Straight path along +X from 0 to length_m with step_m spacing.
static std::vector<Pose2D> straightPath(double length_m, double step_m = 1.0)
{
  std::vector<Pose2D> path;
  for (double x = 0.0; x <= length_m + 1e-9; x += step_m) {
    path.push_back({x, 0.0});
  }
  return path;
}

// Dense rectangular wall (1 cm spacing) in map frame.
static std::vector<ObstaclePoint> wall(double x, double y_min, double y_max, double step = 0.05)
{
  std::vector<ObstaclePoint> obs;
  for (double y = y_min; y <= y_max + 1e-9; y += step) obs.push_back({x, y});
  return obs;
}

// Default avoidance-enabled params with sensible defaults for tests.
static PlannerParams avoidanceParams()
{
  PlannerParams p;
  p.obstacle_avoidance_enabled = true;
  p.max_speed_mps              = 1.0;
  p.lookahead_dist_m           = 3.0;
  p.goal_tolerance_m           = 0.5;
  p.min_approach_linear_velocity = 0.3;
  p.robot_radius_m             = 0.30;
  p.min_turn_radius_m          = 1.0;
  p.local_grid_size_m          = 8.0;
  p.local_grid_resolution_m    = 0.1;
  p.inflate_margin_m           = 0.10;
  p.tentacle_length_forward_m  = 3.0;
  p.tentacle_length_reverse_m  = 1.5;
  p.tentacle_sample_step_m     = 0.1;
  p.num_tentacles_per_side     = 6;
  p.r_stop_hard_m              = 0.45;
  p.r_stop_m                   = 0.65;
  p.r_slow_m                   = 1.30;
  p.v_escape_mps               = 0.4;
  p.t_escape_max_s             = 1.0;   // short for tests
  p.max_escape_attempts        = 3;
  p.tick_period_s              = 0.05;
  return p;
}

// ---------------------------------------------------------------------------
// Pure-pursuit fallback (avoidance disabled) — regression coverage
// ---------------------------------------------------------------------------

TEST(ApproachVelocityScaling, FullSpeedFarFromGoal)
{
  PlannerParams p;
  p.max_speed_mps = 2.0;
  p.lookahead_dist_m = 3.0;
  p.min_approach_linear_velocity = 0.5;
  p.goal_tolerance_m = 0.5;

  auto algo = makeAlgo(p);
  algo.setPath(straightPath(20.0));

  auto res = algo.compute(0.0, 0.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(res.approach_velocity_scale, 1.0);
  EXPECT_NEAR(res.linear_vel, p.max_speed_mps, 1e-9);
}

TEST(ApproachVelocityScaling, ReducedSpeedInsideWindow)
{
  PlannerParams p;
  p.max_speed_mps = 2.0;
  p.lookahead_dist_m = 4.0;
  p.min_approach_linear_velocity = 0.5;
  p.goal_tolerance_m = 0.2;

  auto algo = makeAlgo(p);
  algo.setPath({{0.0, 0.0}, {2.0, 0.0}, {2.0, 0.0}});

  auto res = algo.compute(0.0, 0.0, 0.0, 0.0);
  EXPECT_NEAR(res.approach_velocity_scale, 0.5, 1e-6);
  EXPECT_NEAR(res.linear_vel, 1.25, 1e-6);
}

TEST(VelocityScaledLookahead, MaxSpeedDoublesDist)
{
  PlannerParams p;
  p.lookahead_dist_m = 3.0;
  p.max_speed_mps = 1.5;
  p.goal_tolerance_m = 0.5;

  auto algo = makeAlgo(p);
  algo.setPath(straightPath(20.0));

  auto res = algo.compute(0.0, 0.0, 0.0, p.max_speed_mps);
  EXPECT_NEAR(res.effective_lookahead_dist, 6.0, 1e-9);
}

TEST(InterpolatedCarrot, CoarsePathInterpolated)
{
  PlannerParams p;
  p.lookahead_dist_m = 2.5;
  p.max_speed_mps = 1.5;
  p.goal_tolerance_m = 0.5;

  auto algo = makeAlgo(p);
  algo.setPath({{0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}});

  auto res = algo.compute(0.0, 0.0, 0.0, 0.0);
  EXPECT_TRUE(res.lookahead_interpolated);
  const double actual_dist = std::hypot(res.lookahead_x, res.lookahead_y);
  EXPECT_NEAR(actual_dist, res.effective_lookahead_dist, 1e-6);
}

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

TEST(Regression, StraightAheadZeroSteeringPureP)
{
  PlannerParams p;
  p.lookahead_dist_m = 3.0;
  p.k_p_steering = 1.5;
  p.goal_tolerance_m = 0.5;
  p.max_speed_mps = 1.5;

  auto algo = makeAlgo(p);
  algo.setPath(straightPath(20.0));

  auto res = algo.compute(0.0, 0.0, 0.0, 0.0);
  EXPECT_NEAR(res.heading_err, 0.0, 1e-6);
  EXPECT_NEAR(res.angular_vel, 0.0, 1e-6);
  EXPECT_NEAR(res.linear_vel, p.max_speed_mps, 1e-6);
}

// ---------------------------------------------------------------------------
// Tentacle library shape
// ---------------------------------------------------------------------------

TEST(Tentacles, BuiltWithBothDirections)
{
  auto algo = makeAlgo(avoidanceParams());
  algo.setPath(straightPath(20.0));
  algo.updateObstacles({});

  // Trigger build via compute().
  (void)algo.compute(0.0, 0.0, 0.0, 0.0);

  const auto& ts = algo.tentacles();
  const auto p   = algo.getParams();
  const int expected = 2 * (2 * p.num_tentacles_per_side + 1);
  ASSERT_EQ(static_cast<int>(ts.size()), expected);

  int fwd = 0, rev = 0;
  for (const auto& t : ts) {
    if (t.direction > 0) ++fwd; else ++rev;
    ASSERT_FALSE(t.samples.empty());
    EXPECT_NEAR(t.samples.front().x, 0.0, 1e-9);
    EXPECT_NEAR(t.samples.front().y, 0.0, 1e-9);
  }
  EXPECT_EQ(fwd, expected / 2);
  EXPECT_EQ(rev, expected / 2);
}

TEST(Tentacles, ForwardSampleAtExpectedKinematicEnd)
{
  // A straight forward tentacle (curvature 0) must end at (L, 0).
  auto algo = makeAlgo(avoidanceParams());
  algo.setPath(straightPath(20.0));
  algo.updateObstacles({});
  (void)algo.compute(0.0, 0.0, 0.0, 0.0);

  const auto& ts = algo.tentacles();
  const auto p   = algo.getParams();

  // Find the straight forward tentacle.
  const Tentacle* straight = nullptr;
  for (const auto& t : ts) {
    if (t.direction > 0 && std::fabs(t.curvature) < 1e-9) { straight = &t; break; }
  }
  ASSERT_NE(straight, nullptr);
  EXPECT_NEAR(straight->samples.back().x, p.tentacle_length_forward_m, 0.15);
  EXPECT_NEAR(straight->samples.back().y, 0.0, 1e-6);
}

// ---------------------------------------------------------------------------
// Tentacle behaviour: clear, wall, near-stop
// ---------------------------------------------------------------------------

TEST(Avoidance, PicksStraightTentacleWhenClear)
{
  auto algo = makeAlgo(avoidanceParams());
  algo.setPath(straightPath(20.0));
  algo.updateObstacles({});

  auto res = algo.compute(0.0, 0.0, 0.0, 0.0);
  EXPECT_EQ(res.nav_state, NavState::NAVIGATE);
  EXPECT_NEAR(res.chosen_curvature, 0.0, 1e-9);
  EXPECT_GT(res.linear_vel, 0.0);
  EXPECT_NEAR(res.angular_vel, 0.0, 1e-9);
}

TEST(Avoidance, SteersAroundWallInFront)
{
  auto algo = makeAlgo(avoidanceParams());
  algo.setPath(straightPath(20.0));
  // Wall at x=2.0 spanning y ∈ [-2, 2] — blocks the straight tentacle but
  // leaves the side arcs clear.
  algo.updateObstacles(wall(2.0, -2.0, 2.0));

  auto res = algo.compute(0.0, 0.0, 0.0, 0.0);
  // We expect a curved tentacle (non-zero curvature) and forward motion.
  EXPECT_GT(res.linear_vel, 0.0);
  EXPECT_GT(std::fabs(res.chosen_curvature), 0.05);
  EXPECT_EQ(res.chosen_direction, +1.0);
}

TEST(Avoidance, HardSafetyStopWhenObstacleVeryClose)
{
  auto algo = makeAlgo(avoidanceParams());
  algo.setPath(straightPath(20.0));
  // Obstacle 0.30 m in front — closer than r_stop_hard (=0.45 m).
  algo.updateObstacles({{0.30, 0.0}});

  auto res = algo.compute(0.0, 0.0, 0.0, 0.0);
  EXPECT_NEAR(res.linear_vel, 0.0, 1e-9);
  EXPECT_NEAR(res.angular_vel, 0.0, 1e-9);
  EXPECT_GT(res.closest_r, 0.0);
  EXPECT_LT(res.closest_r, avoidanceParams().r_stop_hard_m);
}

// ---------------------------------------------------------------------------
// FSM: NAVIGATE → PROBE → ESCAPE with reverse availability
// ---------------------------------------------------------------------------

TEST(Fsm, EntersEscapeWhenForwardBlockedAndReverseClear)
{
  auto algo = makeAlgo(avoidanceParams());
  algo.setPath(straightPath(20.0));
  // Tight half-ring of obstacles in front and to both sides at ~1 m, but
  // nothing behind — reverse should remain a viable arc.
  std::vector<ObstaclePoint> obs;
  for (double a = -M_PI / 2.0; a <= M_PI / 2.0 + 1e-6; a += 0.05) {
    obs.push_back({1.0 * std::cos(a), 1.0 * std::sin(a)});
  }
  algo.updateObstacles(obs);

  // Tick 1: NAVIGATE sees forward blocked → transitions into PROBE.
  auto r1 = algo.compute(0.0, 0.0, 0.0, 0.0);
  EXPECT_EQ(r1.nav_state, NavState::PROBE);
  EXPECT_NEAR(r1.linear_vel, 0.0, 1e-9);

  // Tick 2: PROBE sees reverse clear → enters ESCAPE, commands reverse motion.
  auto r2 = algo.compute(0.0, 0.0, 0.0, 0.0);
  EXPECT_EQ(r2.nav_state, NavState::ESCAPE);
  EXPECT_LT(r2.linear_vel, 0.0);
  EXPECT_EQ(r2.chosen_direction, -1.0);
}

TEST(Fsm, EscapeTimeoutEntersReplan)
{
  auto p = avoidanceParams();
  p.t_escape_max_s = 0.1;  // 2 ticks at 50 ms
  auto algo = makeAlgo(p);
  algo.setPath(straightPath(20.0));

  // Box the rover in on all sides ~0.9 m away (inside r_slow but outside
  // r_stop_hard) so PROBE picks ESCAPE but reverse never opens up forward.
  std::vector<ObstaclePoint> obs;
  for (double a = 0.0; a < 2.0 * M_PI; a += 0.04) {
    obs.push_back({0.9 * std::cos(a), 0.9 * std::sin(a)});
  }
  algo.updateObstacles(obs);

  // Drive ticks until REPLAN. Should happen well within ~20 ticks.
  bool saw_replan = false;
  for (int i = 0; i < 30; ++i) {
    auto r = algo.compute(0.0, 0.0, 0.0, 0.0);
    if (r.nav_state == NavState::REPLAN) {
      EXPECT_TRUE(r.request_replan);
      EXPECT_NEAR(r.linear_vel, 0.0, 1e-9);
      saw_replan = true;
      break;
    }
  }
  EXPECT_TRUE(saw_replan);
}

TEST(Fsm, NewPathResetsFsm)
{
  auto p = avoidanceParams();
  p.t_escape_max_s = 0.1;
  auto algo = makeAlgo(p);
  algo.setPath(straightPath(20.0));

  std::vector<ObstaclePoint> obs;
  for (double a = 0.0; a < 2.0 * M_PI; a += 0.04) {
    obs.push_back({0.9 * std::cos(a), 0.9 * std::sin(a)});
  }
  algo.updateObstacles(obs);

  // Drive into REPLAN.
  for (int i = 0; i < 30; ++i) algo.compute(0.0, 0.0, 0.0, 0.0);
  ASSERT_EQ(algo.fsm().state, NavState::REPLAN);

  // New path + clear surroundings → FSM should reset, NAVIGATE again.
  algo.setPath(straightPath(20.0));
  algo.updateObstacles({});
  auto r = algo.compute(0.0, 0.0, 0.0, 0.0);
  EXPECT_EQ(r.nav_state, NavState::NAVIGATE);
  EXPECT_FALSE(r.request_replan);
  EXPECT_GT(r.linear_vel, 0.0);
}
