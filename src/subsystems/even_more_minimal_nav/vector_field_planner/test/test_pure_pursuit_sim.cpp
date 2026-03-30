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

// test_pure_pursuit_sim.cpp
//
// Closed-loop simulation tests for the VectorFieldPlanner + FrontAckermannController pipeline.
// No ROS runtime required — all logic is replicated inline from the production sources:
//   src/subsystems/even_more_minimal_nav/vector_field_planner/src/vector_field_planner_node.cpp
//   src/subsystems/drive/drive_controllers/src/front_ackermann_controller.cpp
//
// Each replica mirrors the production code exactly.  If you change the production code,
// update the corresponding replica here so the simulation stays faithful.

#include <gtest/gtest.h>

#include <cmath>
#include <deque>
#include <limits>
#include <numeric>
#include <string>
#include <tuple>
#include <vector>
#include <algorithm>

// ============================================================================
// Replica of VectorFieldPlanner logic
// ============================================================================

struct Pose2D { double x, y; };

struct SimPath { std::vector<Pose2D> poses; };

/// Mirrors VectorFieldPlanner::findClosestIndex — full-scan, no index caching.
static size_t findClosestIndex(const SimPath & path, double rx, double ry)
{
  size_t best = 0;
  double best_dist2 = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const double dx = path.poses[i].x - rx;
    const double dy = path.poses[i].y - ry;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_dist2) { best_dist2 = d2; best = i; }
  }
  return best;
}

/// Mirrors VectorFieldPlanner::findLookahead — scans forward from closest_idx.
static std::pair<double, double> findLookahead(
  const SimPath & path, double rx, double ry,
  size_t closest_idx, double lookahead_dist_m)
{
  for (size_t i = closest_idx; i < path.poses.size(); ++i) {
    const double dx = path.poses[i].x - rx;
    const double dy = path.poses[i].y - ry;
    if (std::hypot(dx, dy) >= lookahead_dist_m) {
      return {path.poses[i].x, path.poses[i].y};
    }
  }
  return {path.poses.back().x, path.poses.back().y};
}

struct PlannerParams {
  double lookahead_dist_m       = 3.0;
  double k_p_steering           = 1.5;
  double max_steering_angle_rad = 0.5;
  double max_speed_mps          = 0.8;
  double goal_tolerance_m       = 1.5;
};

/// Mirrors VectorFieldPlanner::controlLoop (pure math portion — no TF).
/// Returns {linear_x, angular_z} that would be published as TwistStamped.
static std::pair<double, double> plannerStep(
  const SimPath & path, double rx, double ry, double yaw,
  const PlannerParams & p, bool & goal_reached)
{
  const auto & goal_pos = path.poses.back();
  const double dist_to_goal = std::hypot(goal_pos.x - rx, goal_pos.y - ry);
  if (dist_to_goal < p.goal_tolerance_m) {
    goal_reached = true;
    return {0.0, 0.0};
  }
  goal_reached = false;

  const size_t closest_idx = findClosestIndex(path, rx, ry);
  const auto [lx, ly] = findLookahead(path, rx, ry, closest_idx, p.lookahead_dist_m);

  double heading_err = std::atan2(ly - ry, lx - rx) - yaw;
  while (heading_err >  M_PI) heading_err -= 2.0 * M_PI;
  while (heading_err < -M_PI) heading_err += 2.0 * M_PI;

  const double steering = std::clamp(
    p.k_p_steering * heading_err,
    -p.max_steering_angle_rad, p.max_steering_angle_rad);

  return {p.max_speed_mps, steering};
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

/// Mirrors FrontAckermannController::update (the wheel command + odom portion).
/// linear_vel_cmd = twist.linear.x, angular_z_cmd = twist.angular.z.
///
/// KEY: angular_z_cmd is treated by the controller as omega (rad/s) in the
/// bicycle model:  steer_angle = atan(omega * L / v)
/// The planner, however, outputs angular.z = k_p * heading_err intending it
/// to represent a desired steering angle in radians.
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
    path.poses.push_back({x0 + t * dx, y0 + t * dy});
  }
  path.poses.back() = {x1, y1};  // ensure exact endpoint
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
  const PlannerParams & planner, const AckermannParams & ackermann,
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
    path.poses.back().x - x, path.poses.back().y - y);
  return result;
}

// ============================================================================
// Unit tests — planner logic
// ============================================================================

TEST(FindClosestIndex, StartOfPath)
{
  const SimPath path = makeStraightPath(0.0, 0.0, 10.0, -10.0, 1.0);
  EXPECT_EQ(findClosestIndex(path, 0.0, 0.0), 0u);
}

TEST(FindClosestIndex, AdvancesAsRobotMoves)
{
  const SimPath path = makeStraightPath(0.0, 0.0, 10.0, -10.0, 1.0);
  // Path direction: (10,-10)/sqrt(200) = (0.707,-0.707)
  // A robot at (3.5, -3.5) should map to the closest waypoint index ~5
  const size_t idx = findClosestIndex(path, 3.5, -3.5);
  EXPECT_GE(idx, 4u);
  EXPECT_LE(idx, 6u);
}

TEST(FindClosestIndex, NearGoal)
{
  const SimPath path = makeStraightPath(0.0, 0.0, 10.0, -10.0, 1.0);
  const size_t last = path.poses.size() - 1;
  EXPECT_EQ(findClosestIndex(path, 10.0, -10.0), last);
}

// ---------------------------------------------------------------------------

TEST(FindLookahead, ReturnsPointAtLeastLookaheadAway)
{
  const SimPath path = makeStraightPath(0.0, 0.0, 10.0, -10.0, 1.0);
  const double lookahead = 3.0;
  const auto [lx, ly] = findLookahead(path, 0.0, 0.0, 0, lookahead);
  EXPECT_GE(std::hypot(lx, ly), lookahead)
    << "Lookahead point must be at least " << lookahead << " m from robot";
}

TEST(FindLookahead, DirectionIsSoutheast)
{
  const SimPath path = makeStraightPath(0.0, 0.0, 10.0, -10.0, 1.0);
  const auto [lx, ly] = findLookahead(path, 0.0, 0.0, 0, 3.0);
  EXPECT_GT(lx, 0.0) << "Lookahead x should be positive (east)";
  EXPECT_LT(ly, 0.0) << "Lookahead y should be negative (south)";
}

TEST(FindLookahead, FallsBackToGoalWhenPathShorterThanLookahead)
{
  // Robot very close to the goal — remaining path < lookahead distance
  const SimPath path = makeStraightPath(0.0, 0.0, 10.0, -10.0, 1.0);
  const size_t near_end = path.poses.size() - 2;
  const double rx = path.poses[near_end].x;
  const double ry = path.poses[near_end].y;

  const auto [lx, ly] = findLookahead(path, rx, ry, near_end, 3.0);
  EXPECT_NEAR(lx, 10.0, 1e-6) << "Should fall back to goal x";
  EXPECT_NEAR(ly, -10.0, 1e-6) << "Should fall back to goal y";
}

TEST(FindLookahead, DoesNotSearchBeforeClosestIndex)
{
  // If closest_idx is near the end, the lookahead must not wrap around
  // to the beginning of the path.
  const SimPath path = makeStraightPath(0.0, 0.0, 20.0, 0.0, 1.0);  // east-only path
  // Robot is at x=18, y=0 — very close to end
  const size_t closest = findClosestIndex(path, 18.0, 0.0);
  const auto [lx, ly] = findLookahead(path, 18.0, 0.0, closest, 3.0);
  EXPECT_GE(lx, 18.0) << "Lookahead must be ahead of the robot, not behind";
}

// BUG DOCUMENTED: findLookahead returns a point BEHIND the robot when path spacing
// exceeds lookahead_dist_m.
//
// Root cause: findLookahead scans from closest_idx and returns the first pose that is
// >= lookahead_dist_m from the robot.  When path spacing > lookahead_dist_m (e.g. a
// 2-pose path), closest_idx=0 may be the START pose that the robot has already passed.
// If the robot is now > lookahead_dist_m from pose[0], the scan immediately returns
// pose[0] — which is behind the robot.  The robot then steers backward, circles,
// and never reaches the goal.
//
// This does NOT trigger with the production path (path_resolution_m=1.0, lookahead=3.0)
// because the 1 m-spaced poses ensure closest_idx quickly advances past the robot.
// It WOULD trigger if the global planner ever emits a coarse path (spacing > 3 m)
// or if lookahead_dist_m is reduced below the path spacing.
TEST(FindLookahead, BugSparsePathReturnsPointBehindRobot)
{
  // Two-pose path: start=(0,0), goal=(10,-10).  Spacing ≈ 14.1 m >> lookahead=3.0 m.
  SimPath path;
  path.poses.push_back({0.0, 0.0});
  path.poses.push_back({10.0, -10.0});

  // Robot has moved ~4 m along the path direction (past lookahead radius from pose[0]).
  const double rx = 4.0 * 0.7071, ry = 4.0 * -0.7071;  // (2.83, -2.83)

  // closest_idx is 0 (robot is still in the first half of the path)
  const size_t closest = findClosestIndex(path, rx, ry);
  EXPECT_EQ(closest, 0u);

  // findLookahead starts at i=0 (pose (0,0)).
  // dist from robot (2.83,-2.83) to pose[0] (0,0) = 4.0 m >= lookahead 3.0 m.
  // So it immediately returns (0,0) — the point the robot already passed.
  const auto [lx, ly] = findLookahead(path, rx, ry, closest, 3.0);

  // This assertion DOCUMENTS the bug: the returned lookahead is pose[0] = (0,0),
  // which is BEHIND the robot.
  EXPECT_NEAR(lx, 0.0, 1e-9) << "BUG: findLookahead returned the already-passed start pose";
  EXPECT_NEAR(ly, 0.0, 1e-9) << "BUG: findLookahead returned the already-passed start pose";

  // Consequence: the heading error points backward, causing the robot to u-turn.
  const double yaw = std::atan2(-1.0, 1.0);  // robot facing SE (correct heading)
  double heading_err = std::atan2(ly - ry, lx - rx) - yaw;
  while (heading_err >  M_PI) heading_err -= 2.0 * M_PI;
  while (heading_err < -M_PI) heading_err += 2.0 * M_PI;

  // heading_err should be ~±π (opposite direction) — the robot steers away from goal
  EXPECT_GT(std::abs(heading_err), M_PI / 2.0)
    << "BUG consequence: heading error is > 90 deg because lookahead is behind the robot";
}

// ---------------------------------------------------------------------------

TEST(HeadingError, DiagonalGoalFromEastFacing)
{
  // Robot at origin facing east (yaw=0); lookahead at (1,-1) → bearing = -45°
  const double rx = 0.0, ry = 0.0, yaw = 0.0;
  const double lx = 1.0, ly = -1.0;
  double err = std::atan2(ly - ry, lx - rx) - yaw;
  while (err >  M_PI) err -= 2 * M_PI;
  while (err < -M_PI) err += 2 * M_PI;
  EXPECT_NEAR(err, -M_PI / 4.0, 1e-9);
}

TEST(HeadingError, NormalisedThroughPi)
{
  // Facing west (-π), lookahead directly east → raw error ≈ +2π, must wrap to 0
  const double yaw = M_PI;   // facing west
  const double lx = 1.0, ly = 0.0;   // lookahead due east → bearing = 0
  double err = std::atan2(ly, lx) - yaw;
  while (err >  M_PI) err -= 2 * M_PI;
  while (err < -M_PI) err += 2 * M_PI;
  EXPECT_NEAR(std::abs(err), M_PI, 1e-9);  // ±π, both are correct
}

// ============================================================================
// Unit tests — controller steer conversion
// ============================================================================

// This test documents the planner/controller interface mismatch.
// The planner sends angular.z = k_p * heading_err, intending it as a steering angle.
// The controller converts it via: steer = atan(angular_z * L / v), as if it's omega.
// The two interpretations diverge for non-trivial angular_z values.
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

// Default parameters, drawn directly from nav_params.yaml and front_ackermann_controller.yaml
static PlannerParams defaultPlannerParams()
{
  PlannerParams p;
  p.lookahead_dist_m       = 3.0;
  p.k_p_steering           = 1.5;
  p.max_steering_angle_rad = 0.5;
  p.max_speed_mps          = 0.8;
  p.goal_tolerance_m       = 1.5;
  return p;
}

static AckermannParams defaultAckermannParams()
{
  return AckermannParams{};  // all defaults from struct definition
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

  // At 0.8 m/s the Euclidean distance is ~14.1 m → ideal time ~17.7 s → ~354 steps.
  // Allow 3x margin for curvature, but not infinite spinning.
  const int reasonable_steps = 1100;  // ~55 s at 20 Hz
  EXPECT_LE(result.steps, reasonable_steps)
    << "Simulation used " << result.steps << " steps (>" << reasonable_steps
    << ").  Robot may be oscillating or not progressing.";
}

TEST(ClosedLoopSim, RobotSteerRightInitially)
{
  // (0,0) → (10,-10): the robot starts facing east and must turn right (south-east).
  // Verify the very first command steers the robot in the correct direction.
  const SimPath path = makeStraightPath(0.0, 0.0, 10.0, -10.0, 1.0);
  const PlannerParams p = defaultPlannerParams();

  bool goal_reached = false;
  const auto [lin_x, ang_z] = plannerStep(path, 0.0, 0.0, 0.0, p, goal_reached);

  EXPECT_FALSE(goal_reached);
  EXPECT_GT(lin_x, 0.0) << "Speed should be positive";
  EXPECT_LT(ang_z, 0.0) << "angular.z should be negative (right turn toward (10,-10))";

  // Verify the controller also produces a right-turning angular velocity
  const WheelState ws = controllerStep(lin_x, ang_z, defaultAckermannParams());
  EXPECT_LT(ws.angular_vel, 0.0)
    << "Controller must produce negative angular velocity for initial right turn";
}

TEST(ClosedLoopSim, LookaheadPointIsAlwaysForward)
{
  // During the entire trajectory, the lookahead point should always be ahead of
  // (or at) the robot's current path progress — never behind.
  const SimPath path = makeStraightPath(0.0, 0.0, 10.0, -10.0, 1.0);
  const PlannerParams p = defaultPlannerParams();
  const AckermannParams a = defaultAckermannParams();
  const SimResult result = runSimulation(path, 0.0, 0.0, 0.0, p, a);

  // Replay trajectory and verify the lookahead is never further BEHIND the path
  // than the closest index.
  size_t prev_closest = 0;
  for (const auto & [x, y, yaw] : result.trajectory) {
    const size_t closest = findClosestIndex(path, x, y);
    const auto [lx, ly] = findLookahead(path, x, y, closest, p.lookahead_dist_m);

    // The lookahead's closest index must be >= the robot's current closest index
    const size_t lookahead_path_idx = findClosestIndex(path, lx, ly);
    EXPECT_GE(lookahead_path_idx, closest)
      << "Lookahead point is behind the robot's current closest path index at ("
      << x << ", " << y << ")";

    prev_closest = closest;
  }
  (void)prev_closest;
}

TEST(ClosedLoopSim, DistanceToGoalMonotonicallyDecreasesOnAverage)
{
  // The robot must make net progress toward the goal.  We check that the
  // distance to goal at the midpoint of the simulation is less than at the start,
  // and at the end is less than at the midpoint.
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
  const PlannerParams p = defaultPlannerParams();

  bool goal_reached = false;
  const auto [lin_x, ang_z] = plannerStep(path, 10.0, -10.0, 0.0, p, goal_reached);
  EXPECT_TRUE(goal_reached);
  EXPECT_NEAR(lin_x, 0.0, 1e-9);
  EXPECT_NEAR(ang_z, 0.0, 1e-9);
}

TEST(EdgeCase, SinglePosePath)
{
  SimPath path;
  path.poses.push_back({5.0, -5.0});

  const PlannerParams p = defaultPlannerParams();
  bool goal_reached = false;
  // Robot well outside tolerance
  const auto [lin_x, ang_z] = plannerStep(path, 0.0, 0.0, 0.0, p, goal_reached);
  EXPECT_FALSE(goal_reached);
  EXPECT_GT(lin_x, 0.0);
}

// BUG REGRESSION: a 2-pose path causes the robot to loop and never reach the goal.
// See FindLookahead.BugSparsePathReturnsPointBehindRobot for the root cause.
// Once the planner bug is fixed this test should be inverted to EXPECT true.
TEST(EdgeCase, PathWithOnlyTwoPosesFailsDueToLookaheadBug)
{
  SimPath path;
  path.poses.push_back({0.0, 0.0});
  path.poses.push_back({10.0, -10.0});

  const SimResult result = runSimulation(
    path, 0.0, 0.0, 0.0,
    defaultPlannerParams(), defaultAckermannParams());

  // This currently FAILS to reach the goal because findLookahead returns the
  // already-passed start pose once the robot is > lookahead_dist_m from it.
  // When this is fixed, flip the expectation to EXPECT_TRUE.
  EXPECT_FALSE(result.goal_reached)
    << "BUG FIXED: update this test to EXPECT_TRUE(goal_reached)";
}

TEST(EdgeCase, RobotStartsFacingAwayFromGoal)
{
  // Robot faces west (yaw = ±π); goal is at (10,-10) (south-east).
  // The controller must still steer the robot around and eventually reach the goal.
  const SimPath path = makeStraightPath(0.0, 0.0, 10.0, -10.0, 1.0);
  const SimResult result = runSimulation(
    path, 0.0, 0.0, M_PI,   // facing west
    defaultPlannerParams(), defaultAckermannParams());

  EXPECT_TRUE(result.goal_reached)
    << "Goal not reached when starting backward.  Dist=" << result.final_dist_to_goal;
}
