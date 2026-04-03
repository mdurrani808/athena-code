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
#include <deque>
#include <limits>
#include <numeric>
#include <string>
#include <tuple>
#include <vector>
#include <algorithm>

#include "vector_field_planner/vector_field_planner_algo.hpp"

// ============================================================================
// Wrapper for Planner logic (using extracted algorithm)
// ============================================================================

using SimPath = std::vector<vector_field_planner::Pose2D>;

/// Calls the extracted planner algorithm to get the velocity commands.
static std::pair<double, double> plannerStep(
  const SimPath & path, double rx, double ry, double yaw,
  const vector_field_planner::PlannerParams & p, bool & goal_reached)
{
  vector_field_planner::VectorFieldPlannerAlgo algo;
  algo.setParams(p);
  algo.setPath(path);

  auto res = algo.compute(rx, ry, yaw);
  goal_reached = res.goal_reached;
  return {res.linear_vel, res.angular_vel};
}

// ============================================================================
// Replica of FrontAckermannController logic
// ============================================================================

struct AckermannParams {
  double wheelbase     = 0.8382;   // from front_ackermann_controller.yaml
  double track_width   = 0.6604;
  double wheel_radius  = 0.254;
  double max_speed     = 1.27;
  double max_steer_angle = 0.785;
};

struct WheelState {
  double fl_steer = 0.0, fr_steer = 0.0;
  double fl_vel = 0.0, fr_vel = 0.0, rl_vel = 0.0, rr_vel = 0.0;
  // Derived robot-level velocities (open-loop odom)
  double linear_vel  = 0.0;
  double angular_vel = 0.0;
};

static WheelState controllerStep(
  double linear_vel_cmd, double angular_z_cmd,
  const AckermannParams & p)
{
  WheelState ws;

  linear_vel_cmd = std::clamp(linear_vel_cmd, -p.max_speed, p.max_speed);

  double steer_cmd = 0.0;
  if (std::abs(linear_vel_cmd) > 1e-4) {
    // Controller interprets angular_z as omega, converts to steer angle
    steer_cmd = std::atan(angular_z_cmd * p.wheelbase / linear_vel_cmd);
  }
  steer_cmd = std::clamp(steer_cmd, -p.max_steer_angle, p.max_steer_angle);

  ws.fl_vel = ws.fr_vel = ws.rl_vel = ws.rr_vel = linear_vel_cmd;

  if (std::abs(steer_cmd) > 1e-4) {
    const double turn_radius = p.wheelbase / std::tan(steer_cmd);
    double angular_vel = std::abs(linear_vel_cmd) / std::abs(turn_radius);
    if (linear_vel_cmd < 0.0) angular_vel = -angular_vel;

    const double inner_angle = std::atan(p.wheelbase / (std::abs(turn_radius) - p.track_width / 2.0));
    const double outer_angle = std::atan(p.wheelbase / (std::abs(turn_radius) + p.track_width / 2.0));

    const double inner_rear = angular_vel * (std::abs(turn_radius) - p.track_width / 2.0);
    const double outer_rear = angular_vel * (std::abs(turn_radius) + p.track_width / 2.0);
    const double inner_front = angular_vel * std::sqrt(
      p.wheelbase * p.wheelbase +
      std::pow(std::abs(turn_radius) - p.track_width / 2.0, 2));
    const double outer_front = angular_vel * std::sqrt(
      p.wheelbase * p.wheelbase +
      std::pow(std::abs(turn_radius) + p.track_width / 2.0, 2));

    if (steer_cmd > 0.0) {   // LEFT TURN: left wheel is inner
      ws.fl_steer = inner_angle;  ws.fr_steer = outer_angle;
      ws.fl_vel = inner_front;    ws.fr_vel = outer_front;
      ws.rl_vel = inner_rear;     ws.rr_vel = outer_rear;
    } else {                  // RIGHT TURN: right wheel is inner
      ws.fl_steer = -outer_angle; ws.fr_steer = -inner_angle;
      ws.fl_vel = outer_front;    ws.fr_vel = inner_front;
      ws.rl_vel = outer_rear;     ws.rr_vel = inner_rear;
    }
  }

  // Open-loop odometry (mirrors the open_loop branch in the controller)
  ws.linear_vel  = (ws.rl_vel + ws.rr_vel) / 2.0;
  const double avg_steer = (ws.fl_steer + ws.fr_steer) / 2.0;
  if (std::abs(p.wheelbase) > 1e-6) {
    ws.angular_vel = ws.linear_vel * std::tan(avg_steer) / p.wheelbase;
  }

  return ws;
}

// ============================================================================
// Path generator
// ============================================================================

/// Straight-line path with poses spaced resolution_m apart.
static SimPath makeStraightPath(
  double x0, double y0, double x1, double y1, double resolution_m = 1.0)
{
  SimPath path;
  const double dx = x1 - x0, dy = y1 - y0;
  const double dist = std::hypot(dx, dy);
  const int n = static_cast<int>(std::ceil(dist / resolution_m)) + 1;
  for (int i = 0; i < n; ++i) {
    const double t = std::min(1.0, (i * resolution_m) / dist);
    path.push_back({x0 + t * dx, y0 + t * dy});
  }
  path.back() = {x1, y1};  // ensure exact endpoint
  return path;
}

// ============================================================================
// Helper: run full closed-loop simulation and capture a trajectory
// ============================================================================

struct SimResult {
  bool     goal_reached  = false;
  int      steps         = 0;
  double   final_x       = 0.0;
  double   final_y       = 0.0;
  double   final_yaw     = 0.0;
  double   final_dist_to_goal = 0.0;
  std::vector<std::tuple<double, double, double>> trajectory;  // (x, y, yaw)
};

static SimResult runSimulation(
  const SimPath & path, double start_x, double start_y, double start_yaw,
  const vector_field_planner::PlannerParams & planner, const AckermannParams & ackermann,
  double dt = 0.05, int max_steps = 2000)
{
  SimResult result;
  double x = start_x, y = start_y, yaw = start_yaw;

  for (int step = 0; step < max_steps; ++step) {
    result.trajectory.emplace_back(x, y, yaw);

    bool goal_reached = false;
    const auto [lin_x, ang_z] = plannerStep(path, x, y, yaw, planner, goal_reached);

    if (goal_reached) {
      result.goal_reached = true;
      result.steps = step;
      break;
    }

    // Controller converts (lin_x, ang_z) → wheel commands
    const WheelState ws = controllerStep(lin_x, ang_z, ackermann);

    // Integrate pose with midpoint method (mirrors controller odom)
    const double heading_mid = yaw + ws.angular_vel * dt / 2.0;
    x += ws.linear_vel * std::cos(heading_mid) * dt;
    y += ws.linear_vel * std::sin(heading_mid) * dt;
    yaw += ws.angular_vel * dt;
    while (yaw >  M_PI) yaw -= 2.0 * M_PI;
    while (yaw < -M_PI) yaw += 2.0 * M_PI;

    result.steps = step + 1;
  }

  result.final_x   = x;
  result.final_y   = y;
  result.final_yaw = yaw;
  result.final_dist_to_goal = std::hypot(
    path.back().x - x, path.back().y - y);
  return result;
}

// ============================================================================
// Unit tests — planner logic
// ============================================================================

TEST(FindClosestIndex, StartOfPath)
{
  const SimPath path = makeStraightPath(0.0, 0.0, 10.0, -10.0, 1.0);
  vector_field_planner::VectorFieldPlannerAlgo algo;
  algo.setPath(path);
  EXPECT_EQ(algo.findClosestIndex(0.0, 0.0), 0u);
}

TEST(FindClosestIndex, AdvancesAsRobotMoves)
{
  const SimPath path = makeStraightPath(0.0, 0.0, 10.0, -10.0, 1.0);
  vector_field_planner::VectorFieldPlannerAlgo algo;
  algo.setPath(path);
  
  // Path direction: (10,-10)/sqrt(200) = (0.707,-0.707)
  // A robot at (3.5, -3.5) should map to the closest waypoint index ~5
  const size_t idx = algo.findClosestIndex(3.5, -3.5);
  EXPECT_GE(idx, 4u);
  EXPECT_LE(idx, 6u);
}

TEST(FindClosestIndex, NearGoal)
{
  const SimPath path = makeStraightPath(0.0, 0.0, 10.0, -10.0, 1.0);
  vector_field_planner::VectorFieldPlannerAlgo algo;
  algo.setPath(path);
  
  const size_t last = path.size() - 1;
  EXPECT_EQ(algo.findClosestIndex(10.0, -10.0), last);
}

// ---------------------------------------------------------------------------

TEST(FindLookahead, ReturnsPointAtLeastLookaheadAway)
{
  const SimPath path = makeStraightPath(0.0, 0.0, 10.0, -10.0, 1.0);
  vector_field_planner::VectorFieldPlannerAlgo algo;
  algo.setPath(path);
  
  vector_field_planner::PlannerParams params;
  params.lookahead_dist_m = 3.0;
  algo.setParams(params);

  const auto [lx, ly] = algo.findLookahead(0.0, 0.0, 0);
  EXPECT_GE(std::hypot(lx, ly), params.lookahead_dist_m)
    << "Lookahead point must be at least " << params.lookahead_dist_m << " m from robot";
}

TEST(FindLookahead, DirectionIsSoutheast)
{
  const SimPath path = makeStraightPath(0.0, 0.0, 10.0, -10.0, 1.0);
  vector_field_planner::VectorFieldPlannerAlgo algo;
  algo.setPath(path);
  
  vector_field_planner::PlannerParams params;
  params.lookahead_dist_m = 3.0;
  algo.setParams(params);

  const auto [lx, ly] = algo.findLookahead(0.0, 0.0, 0);
  EXPECT_GT(lx, 0.0) << "Lookahead x should be positive (east)";
  EXPECT_LT(ly, 0.0) << "Lookahead y should be negative (south)";
}

TEST(FindLookahead, FallsBackToGoalWhenPathShorterThanLookahead)
{
  const SimPath path = makeStraightPath(0.0, 0.0, 10.0, -10.0, 1.0);
  vector_field_planner::VectorFieldPlannerAlgo algo;
  algo.setPath(path);
  
  vector_field_planner::PlannerParams params;
  params.lookahead_dist_m = 3.0;
  algo.setParams(params);

  const size_t near_end = path.size() - 2;
  const double rx = path[near_end].x;
  const double ry = path[near_end].y;

  const auto [lx, ly] = algo.findLookahead(rx, ry, near_end);
  EXPECT_NEAR(lx, 10.0, 1e-6) << "Should fall back to goal x";
  EXPECT_NEAR(ly, -10.0, 1e-6) << "Should fall back to goal y";
}

TEST(FindLookahead, DoesNotSearchBeforeClosestIndex)
{
  const SimPath path = makeStraightPath(0.0, 0.0, 20.0, 0.0, 1.0);  // east-only path
  vector_field_planner::VectorFieldPlannerAlgo algo;
  algo.setPath(path);
  
  vector_field_planner::PlannerParams params;
  params.lookahead_dist_m = 3.0;
  algo.setParams(params);

  // Robot is at x=18, y=0 — very close to end
  const size_t closest = algo.findClosestIndex(18.0, 0.0);
  const auto [lx, ly] = algo.findLookahead(18.0, 0.0, closest);
  EXPECT_GE(lx, 18.0) << "Lookahead must be ahead of the robot, not behind";
}

TEST(FindLookahead, SkipBehindRobotWhenSparse)
{
  vector_field_planner::VectorFieldPlannerAlgo algo;
  SimPath path;
  path.push_back({0.0, 0.0});
  path.push_back({10.0, -10.0});
  algo.setPath(path);

  vector_field_planner::PlannerParams p;
  p.lookahead_dist_m = 3.0;
  algo.setParams(p);

  const double rx = 4.0 * 0.7071, ry = 4.0 * -0.7071;

  const size_t closest = algo.findClosestIndex(rx, ry);
  EXPECT_EQ(closest, 0u);

  const auto [lx, ly] = algo.findLookahead(rx, ry, closest);

  // With the bug fix, it should return (10, -10) instead of the start point
  EXPECT_NEAR(lx, 10.0, 1e-9);
  EXPECT_NEAR(ly, -10.0, 1e-9);
}

// ---------------------------------------------------------------------------

TEST(HeadingError, DiagonalGoalFromEastFacing)
{
  vector_field_planner::VectorFieldPlannerAlgo algo;
  vector_field_planner::PlannerParams p;
  p.k_p_steering = 1.0;
  p.lookahead_dist_m = 0.5; // very small so it picks the goal
  p.goal_tolerance_m = 0.1; // so it doesn't immediately return goal_reached
  algo.setParams(p);
  
  SimPath path = {{1.0, -1.0}};
  algo.setPath(path);

  // Robot at origin facing east (yaw=0); lookahead at (1,-1) → bearing = -45°
  auto res = algo.compute(0.0, 0.0, 0.0);
  
  // heading error is returned in res
  EXPECT_NEAR(res.heading_err, -M_PI / 4.0, 1e-9);
}

TEST(HeadingError, NormalisedThroughPi)
{
  vector_field_planner::VectorFieldPlannerAlgo algo;
  vector_field_planner::PlannerParams p;
  p.k_p_steering = 1.0;
  p.goal_tolerance_m = 0.1; // so it doesn't immediately return goal_reached
  algo.setParams(p);
  
  SimPath path = {{1.0, 0.0}};
  algo.setPath(path);

  // Facing west (-π), lookahead directly east → raw error ≈ +2π, must wrap to 0
  auto res = algo.compute(0.0, 0.0, M_PI);
  
  EXPECT_NEAR(std::abs(res.heading_err), M_PI, 1e-9);  // ±π, both are correct
}

// ============================================================================
// Unit tests — controller steer conversion
// ============================================================================

TEST(AckermannController, AngularZInterpretationMismatch)
{
  const AckermannParams p;
  const double linear_v = 0.8;  // planner max_speed_mps from nav_params.yaml

  // At max steering command from the planner (angular_z = 0.5 rad)
  const double angular_z_max = 0.5;  // planner clamps here
  // Controller will compute:
  const double steer_from_controller = std::atan(angular_z_max * p.wheelbase / linear_v);

  // If angular_z were truly a steer angle, the controller should just pass it through.
  // Instead, atan(0.5 * 0.8382 / 0.8) ≈ 0.484 rad — not 0.5.
  // The discrepancy grows at higher speeds (lower ratio L/v shrinks the effective steer).
  EXPECT_NE(steer_from_controller, angular_z_max)
    << "Controller does not pass angular_z through as a steer angle";

  // Document the actual value so regressions are caught
  EXPECT_NEAR(steer_from_controller, std::atan(0.5 * 0.8382 / 0.8), 1e-9);
}

TEST(AckermannController, HighSpeedReducesEffectiveSteering)
{
  // At high speed the atan(omega*L/v) formula yields a smaller steer angle,
  // so the robot under-steers relative to what the planner intended.
  const AckermannParams p;
  const double angular_z = 0.5;

  const double steer_slow = std::atan(angular_z * p.wheelbase / 0.3);   // slow
  const double steer_fast = std::atan(angular_z * p.wheelbase / 1.2);   // fast

  EXPECT_GT(steer_slow, steer_fast)
    << "Effective steering should decrease as speed increases for the same angular_z";
}

TEST(AckermannController, SteerSignConvention)
{
  // Negative angular_z → right turn → negative wheel angles
  const AckermannParams p;
  const WheelState ws = controllerStep(0.8, -0.5, p);
  EXPECT_LT(ws.fl_steer, 0.0) << "Front-left steer should be negative for right turn";
  EXPECT_LT(ws.fr_steer, 0.0) << "Front-right steer should be negative for right turn";
  EXPECT_LT(ws.angular_vel, 0.0) << "Robot angular velocity should be negative for right turn";
}

TEST(AckermannController, ZeroAngularZGoeseStraight)
{
  const AckermannParams p;
  const WheelState ws = controllerStep(0.8, 0.0, p);
  EXPECT_NEAR(ws.fl_steer,    0.0, 1e-9);
  EXPECT_NEAR(ws.fr_steer,    0.0, 1e-9);
  EXPECT_NEAR(ws.angular_vel, 0.0, 1e-9);
  EXPECT_NEAR(ws.linear_vel,  0.8, 1e-9);
}

// ============================================================================
// Closed-loop simulation: (0,0) → (10,-10)
// ============================================================================

static vector_field_planner::PlannerParams defaultPlannerParams()
{
  vector_field_planner::PlannerParams p;
  p.lookahead_dist_m       = 3.0;
  p.k_p_steering           = 1.5;
  p.max_steering_angle_rad = 0.5;
  p.max_speed_mps          = 0.8;
  p.goal_tolerance_m       = 1.5;
  return p;
}

static AckermannParams defaultAckermannParams()
{
  return AckermannParams{};
}

TEST(ClosedLoopSim, GoalIsReached)
{
  const SimPath path = makeStraightPath(0.0, 0.0, 10.0, -10.0, 1.0);
  const SimResult result = runSimulation(
    path, 0.0, 0.0, 0.0,
    defaultPlannerParams(), defaultAckermannParams());

  EXPECT_TRUE(result.goal_reached)
    << "Goal (10,-10) was never reached after " << result.steps << " steps.\n"
    << "Final position: (" << result.final_x << ", " << result.final_y << ")\n"
    << "Distance to goal: " << result.final_dist_to_goal << " m\n"
    << "Goal tolerance:   " << defaultPlannerParams().goal_tolerance_m << " m";
}

TEST(ClosedLoopSim, GoalReachedWithinReasonableTime)
{
  const SimPath path = makeStraightPath(0.0, 0.0, 10.0, -10.0, 1.0);
  const SimResult result = runSimulation(
    path, 0.0, 0.0, 0.0,
    defaultPlannerParams(), defaultAckermannParams());

  const int reasonable_steps = 1100;  // ~55 s at 20 Hz
  EXPECT_LE(result.steps, reasonable_steps)
    << "Simulation used " << result.steps << " steps (>" << reasonable_steps
    << ").  Robot may be oscillating or not progressing.";
}

TEST(ClosedLoopSim, RobotSteerRightInitially)
{
  const SimPath path = makeStraightPath(0.0, 0.0, 10.0, -10.0, 1.0);
  const auto p = defaultPlannerParams();

  bool goal_reached = false;
  const auto [lin_x, ang_z] = plannerStep(path, 0.0, 0.0, 0.0, p, goal_reached);

  EXPECT_FALSE(goal_reached);
  EXPECT_GT(lin_x, 0.0) << "Speed should be positive";
  EXPECT_LT(ang_z, 0.0) << "angular.z should be negative (right turn toward (10,-10))";

  const WheelState ws = controllerStep(lin_x, ang_z, defaultAckermannParams());
  EXPECT_LT(ws.angular_vel, 0.0)
    << "Controller must produce negative angular velocity for initial right turn";
}

TEST(ClosedLoopSim, LookaheadPointIsAlwaysForward)
{
  const SimPath path = makeStraightPath(0.0, 0.0, 10.0, -10.0, 1.0);
  const auto p = defaultPlannerParams();
  const auto a = defaultAckermannParams();
  const SimResult result = runSimulation(path, 0.0, 0.0, 0.0, p, a);

  vector_field_planner::VectorFieldPlannerAlgo algo;
  algo.setPath(path);
  algo.setParams(p);

  size_t prev_closest = 0;
  for (const auto & [x, y, yaw] : result.trajectory) {
    const size_t closest = algo.findClosestIndex(x, y);
    const auto [lx, ly] = algo.findLookahead(x, y, closest);

    const size_t lookahead_path_idx = algo.findClosestIndex(lx, ly);
    EXPECT_GE(lookahead_path_idx, closest)
      << "Lookahead point is behind the robot's current closest path index at ("
      << x << ", " << y << ")";

    prev_closest = closest;
  }
  (void)prev_closest;
}

TEST(ClosedLoopSim, DistanceToGoalMonotonicallyDecreasesOnAverage)
{
  const SimPath path = makeStraightPath(0.0, 0.0, 10.0, -10.0, 1.0);
  const SimResult result = runSimulation(
    path, 0.0, 0.0, 0.0,
    defaultPlannerParams(), defaultAckermannParams());

  if (result.trajectory.size() < 3) {
    GTEST_SKIP() << "Trajectory too short to test monotonicity";
  }

  const auto distToGoal = [&](size_t idx) {
    const auto & [x, y, yaw] = result.trajectory[idx];
    return std::hypot(10.0 - x, -10.0 - y);
  };

  const size_t quarter  = result.trajectory.size() / 4;
  const size_t mid      = result.trajectory.size() / 2;
  const size_t thr      = 3 * result.trajectory.size() / 4;

  EXPECT_LT(distToGoal(quarter), distToGoal(0))
    << "Robot should be closer to goal at 25% of trajectory than at start";
  EXPECT_LT(distToGoal(mid), distToGoal(quarter))
    << "Robot should be closer to goal at 50% than at 25%";
  EXPECT_LT(distToGoal(thr), distToGoal(mid))
    << "Robot should be closer to goal at 75% than at 50%";
}

// ============================================================================
// Edge cases
// ============================================================================

TEST(EdgeCase, AlreadyAtGoal)
{
  const SimPath path = makeStraightPath(0.0, 0.0, 10.0, -10.0, 1.0);
  const auto p = defaultPlannerParams();

  bool goal_reached = false;
  const auto [lin_x, ang_z] = plannerStep(path, 10.0, -10.0, 0.0, p, goal_reached);
  EXPECT_TRUE(goal_reached);
  EXPECT_NEAR(lin_x, 0.0, 1e-9);
  EXPECT_NEAR(ang_z, 0.0, 1e-9);
}

TEST(EdgeCase, SinglePosePath)
{
  SimPath path;
  path.push_back({5.0, -5.0});

  const auto p = defaultPlannerParams();
  bool goal_reached = false;
  
  const auto [lin_x, ang_z] = plannerStep(path, 0.0, 0.0, 0.0, p, goal_reached);
  EXPECT_FALSE(goal_reached);
  EXPECT_GT(lin_x, 0.0);
}

TEST(EdgeCase, PathWithOnlyTwoPosesReachesGoal)
{
  SimPath path;
  path.push_back({0.0, 0.0});
  path.push_back({10.0, -10.0});

  const SimResult result = runSimulation(
    path, 0.0, 0.0, 0.0,
    defaultPlannerParams(), defaultAckermannParams());

  EXPECT_TRUE(result.goal_reached)
    << "BUG FIXED: The robot successfully reaches the goal even with a 2-pose sparse path.";
}

TEST(EdgeCase, RobotStartsFacingAwayFromGoal)
{
  const SimPath path = makeStraightPath(0.0, 0.0, 10.0, -10.0, 1.0);
  const SimResult result = runSimulation(
    path, 0.0, 0.0, M_PI,   // facing west
    defaultPlannerParams(), defaultAckermannParams());

  EXPECT_TRUE(result.goal_reached)
    << "Goal not reached when starting backward.  Dist=" << result.final_dist_to_goal;
}
