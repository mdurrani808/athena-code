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
#include <tuple>
#include <algorithm>
#include <iostream>

#include "mission_executive/mission_executive_algo.hpp"
#include "global_planner/global_planner_algo.hpp"
#include "vector_field_planner/vector_field_planner_algo.hpp"

// ============================================================================
// Replica of FrontAckermannController logic (from test_pure_pursuit_sim.cpp)
// ============================================================================

struct AckermannParams {
  double wheelbase       = 0.8382;
  double track_width     = 0.6604;
  double wheel_radius    = 0.254;
  double max_speed       = 1.27;
  double max_steer_angle = 0.785;
};

struct WheelState {
  double fl_steer = 0.0, fr_steer = 0.0;
  double fl_vel = 0.0, fr_vel = 0.0, rl_vel = 0.0, rr_vel = 0.0;
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
    const double inner_front = angular_vel * std::sqrt(p.wheelbase * p.wheelbase + std::pow(std::abs(turn_radius) - p.track_width / 2.0, 2));
    const double outer_front = angular_vel * std::sqrt(p.wheelbase * p.wheelbase + std::pow(std::abs(turn_radius) + p.track_width / 2.0, 2));

    if (steer_cmd > 0.0) {   
      ws.fl_steer = inner_angle;  ws.fr_steer = outer_angle;
      ws.fl_vel = inner_front;    ws.fr_vel = outer_front;
      ws.rl_vel = inner_rear;     ws.rr_vel = outer_rear;
    } else {                  
      ws.fl_steer = -outer_angle; ws.fr_steer = -inner_angle;
      ws.fl_vel = outer_front;    ws.fr_vel = inner_front;
      ws.rl_vel = outer_rear;     ws.rr_vel = inner_rear;
    }
  }

  ws.linear_vel  = (ws.rl_vel + ws.rr_vel) / 2.0;
  const double avg_steer = (ws.fl_steer + ws.fr_steer) / 2.0;
  if (std::abs(p.wheelbase) > 1e-6) {
    ws.angular_vel = ws.linear_vel * std::tan(avg_steer) / p.wheelbase;
  }

  return ws;
}

// ============================================================================
// Pipeline Mock
// ============================================================================

struct MockPipeline {
  mission_executive::MissionExecutiveAlgo mission;
  global_planner::GlobalPlannerAlgo global;
  vector_field_planner::VectorFieldPlannerAlgo local;
  AckermannParams ackermann;

  double x = 0.0, y = 0.0, yaw = 0.0;
  double time_s = 0.0;
  double dt = 0.05; // 20 Hz
  
  std::vector<global_planner::Pose2D> current_global_path;

  MockPipeline() {
    // Setup parameters
    mission_executive::MissionParams mp;
    mp.stop_angular_vel_threshold = 0.05;
    mp.arrival_hold_time = 1.0;
    mp.replan_distance_m = 3.0;
    mp.spiral_radius_m = 10.0;
    mission.setParams(mp);

    global_planner::PlannerParams gp;
    gp.path_resolution_m = 1.0;
    gp.use_costmap = false; // Just use straight line for mock
    global.setParams(gp);

    vector_field_planner::PlannerParams vp;
    vp.max_speed_mps = 0.8;
    vp.lookahead_dist_m = 3.0;
    vp.goal_tolerance_m = 1.5; 
    vp.obstacle_avoidance_enabled = false;
    local.setParams(vp);
  }

  void setPose(double px, double py, double pyaw) {
    x = px; y = py; yaw = pyaw;
  }

  void planGlobal(double gx, double gy) {
    current_global_path = global.planStraightLine(x, y, gx, gy);
    
    // Convert to mission and local types
    std::vector<mission_executive::Pose2D> mp;
    std::vector<vector_field_planner::Pose2D> lp;
    for (auto& p : current_global_path) {
      mp.push_back({p.x, p.y, p.yaw});
      lp.push_back({p.x, p.y});
    }
    
    mission.updateGlobalPath(mp);
    local.setPath(lp);
  }

  bool step(int max_steps = 10000) {
    bool goal_reached = false;

    for (int i = 0; i < max_steps; ++i) {
      mission.updateRobotPose({x, y, yaw});
      
      auto tick_res = mission.tick(time_s);
      
      if (tick_res.publish_goal) {
        planGlobal(tick_res.goal_to_publish.x, tick_res.goal_to_publish.y);
      }

      if (tick_res.action_finished) {
        goal_reached = tick_res.action_success;
        if (!tick_res.start_queued_goal) {
          break; // Action is complete
        }
      }

      if (mission.getState() == mission_executive::State::IDLE) {
        break; 
      }

      // If Navigating, get command from Local Planner
      double linear_cmd = 0.0;
      double angular_cmd = 0.0;

      if (mission.isNavEnabled() && mission.getState() != mission_executive::State::ARRIVING) {
         auto local_res = local.compute(x, y, yaw);
         if (!local_res.goal_reached) {
           linear_cmd = local_res.linear_vel;
           angular_cmd = local_res.angular_vel;
         }
      } else if (mission.getState() == mission_executive::State::ARRIVING) {
        // Slow down or stop if arriving
        linear_cmd = 0.0;
        angular_cmd = 0.0;
      }

      // Step Ackermann
      auto ws = controllerStep(linear_cmd, angular_cmd, ackermann);
      
      // Update Pose
      const double heading_mid = yaw + ws.angular_vel * dt / 2.0;
      x += ws.linear_vel * std::cos(heading_mid) * dt;
      y += ws.linear_vel * std::sin(heading_mid) * dt;
      yaw += ws.angular_vel * dt;
      
      while (yaw > M_PI) yaw -= 2.0 * M_PI;
      while (yaw < -M_PI) yaw += 2.0 * M_PI;

      // Update Mission IMU
      mission.updateImu(std::abs(ws.angular_vel), time_s);

      time_s += dt;
    }

    return goal_reached;
  }
};

TEST(MockPipelineTest, FullAutonomousGNSSNavigation) {
  MockPipeline sim;
  sim.setPose(0.0, 0.0, 0.0);

  // 1.f.iii: 2 GNSS-only locations. Stopping within 3m is considered successful.
  // The local planner uses 1.5m goal tolerance by default, which is safely < 3m.

  mission_executive::TargetEntry gnss1{"gnss1", 20.0, 10.0, 0, 0, 2.5, false};
  sim.mission.setTarget(gnss1);

  auto res = sim.mission.startNav("gnss1", std::nullopt, false);
  ASSERT_TRUE(res.accepted);
  if (res.publish_goal) sim.planGlobal(res.goal_to_publish.x, res.goal_to_publish.y);

  bool success = sim.step();
  
  EXPECT_TRUE(success) << "Failed to reach GNSS target 1";
  EXPECT_EQ(sim.mission.getState(), mission_executive::State::STOPPED_AT_TARGET);
  
  double dist = std::hypot(sim.x - 20.0, sim.y - 10.0);
  EXPECT_LE(dist, 3.0) << "Final distance > 3m limit for GNSS point";
}

TEST(MockPipelineTest, ArucoPostWithSpiralSearch) {
  MockPipeline sim;
  sim.setPose(0.0, 0.0, 0.0);

  // 1.f.iv: Posts with AR markers.
  // Target type 1 is ARUCO_POST
  mission_executive::TargetEntry post1{"post1", 10.0, 0.0, 1, 0, 2.0, false};
  sim.mission.setTarget(post1);

  auto res = sim.mission.startNav("post1", std::nullopt, false);
  ASSERT_TRUE(res.accepted);
  if (res.publish_goal) sim.planGlobal(res.goal_to_publish.x, res.goal_to_publish.y);

  // We need to inject a detection while it's spiraling.
  // Run until it enters spiral coverage
  for (int i=0; i<5000; i++) {
    sim.mission.updateRobotPose({sim.x, sim.y, sim.yaw});
    auto tick_res = sim.mission.tick(sim.time_s);
    if (tick_res.publish_goal) sim.planGlobal(tick_res.goal_to_publish.x, tick_res.goal_to_publish.y);
    
    if (sim.mission.getState() == mission_executive::State::SPIRAL_COVERAGE) {
      break;
    }

    double linear_cmd = 0.0, angular_cmd = 0.0;
    if (sim.mission.isNavEnabled() && sim.mission.getState() != mission_executive::State::ARRIVING) {
      auto local_res = sim.local.compute(sim.x, sim.y, sim.yaw);
      if (!local_res.goal_reached) {
        linear_cmd = local_res.linear_vel;
        angular_cmd = local_res.angular_vel;
      }
    }
    
    auto ws = controllerStep(linear_cmd, angular_cmd, sim.ackermann);
    const double heading_mid = sim.yaw + ws.angular_vel * sim.dt / 2.0;
    sim.x += ws.linear_vel * std::cos(heading_mid) * sim.dt;
    sim.y += ws.linear_vel * std::sin(heading_mid) * sim.dt;
    sim.yaw += ws.angular_vel * sim.dt;
    sim.mission.updateImu(std::abs(ws.angular_vel), sim.time_s);
    sim.time_s += sim.dt;
  }

  EXPECT_EQ(sim.mission.getState(), mission_executive::State::SPIRAL_COVERAGE) << "Did not transition to spiral coverage";

  // Simulate CV detection
  sim.mission.onDetection();
  
  // Finish step
  bool success = sim.step();
  EXPECT_TRUE(success);
  EXPECT_EQ(sim.mission.getState(), mission_executive::State::SPIRAL_DONE);
}

TEST(MockPipelineTest, AbortAndReturnToPreviousGNSS) {
  MockPipeline sim;
  sim.setPose(0.0, 0.0, 0.0);

  // 1.f.viii: Operators may abort and return to any *previous* GNSS coordinate.
  mission_executive::TargetEntry start_gate{"start", 0.0, 0.0, 0, 0, 2.0, true};
  sim.mission.setTarget(start_gate);

  mission_executive::TargetEntry gnss1{"gnss1", 30.0, 0.0, 0, 0, 2.0, false};
  sim.mission.setTarget(gnss1);

  // Start moving to gnss1
  auto res = sim.mission.startNav("gnss1", std::nullopt, false);
  ASSERT_TRUE(res.accepted);
  if (res.publish_goal) sim.planGlobal(res.goal_to_publish.x, res.goal_to_publish.y);

  // Step 200 times (10 seconds), rover is moving
  for(int i=0; i<200; i++) {
    sim.mission.updateRobotPose({sim.x, sim.y, sim.yaw});
    auto tick_res = sim.mission.tick(sim.time_s);
    if (tick_res.publish_goal) sim.planGlobal(tick_res.goal_to_publish.x, tick_res.goal_to_publish.y);
    
    auto local_res = sim.local.compute(sim.x, sim.y, sim.yaw);
    auto ws = controllerStep(local_res.linear_vel, local_res.angular_vel, sim.ackermann);
    const double heading_mid = sim.yaw + ws.angular_vel * sim.dt / 2.0;
    sim.x += ws.linear_vel * std::cos(heading_mid) * sim.dt;
    sim.y += ws.linear_vel * std::sin(heading_mid) * sim.dt;
    sim.yaw += ws.angular_vel * sim.dt;
    sim.mission.updateImu(std::abs(ws.angular_vel), sim.time_s);
    sim.time_s += sim.dt;
  }

  // Rover is now somewhere around X=8.0
  EXPECT_GT(sim.x, 2.0);
  EXPECT_LT(sim.x, 15.0);
  EXPECT_EQ(sim.mission.getState(), mission_executive::State::NAVIGATING);

  // Trigger ABORT
  sim.mission.abort();
  EXPECT_EQ(sim.mission.getState(), mission_executive::State::ABORTING);

  // While aborting, queue a RETURN to start_gate
  auto ret_res = sim.mission.startNav("start", std::nullopt, true);
  EXPECT_TRUE(ret_res.accepted);

  // Step to completion
  bool success = sim.step();
  
  EXPECT_TRUE(success) << "Failed to return to start gate";
  EXPECT_EQ(sim.mission.getState(), mission_executive::State::STOPPED_AT_RETURN);
  
  double dist = std::hypot(sim.x - 0.0, sim.y - 0.0);
  EXPECT_LE(dist, 3.0) << "Did not successfully return within tolerance";
}
