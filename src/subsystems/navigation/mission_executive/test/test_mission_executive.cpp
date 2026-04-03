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
#include "mission_executive/mission_executive_algo.hpp"

using namespace mission_executive;

class MissionExecutiveAlgoTest : public ::testing::Test {
protected:
  void SetUp() override {
    MissionParams p;
    p.stop_angular_vel_threshold = 0.05;
    p.arrival_hold_time = 1.0;
    p.replan_distance_m = 2.0;
    p.spiral_timeout_s = 60.0;
    p.spiral_radius_m = 10.0;
    p.spiral_spacing_m = 2.0;
    p.spiral_angular_step = 0.5;
    p.spiral_waypoint_tolerance_m = 1.0;
    algo_.setParams(p);
  }

  MissionExecutiveAlgo algo_;
};

TEST_F(MissionExecutiveAlgoTest, InitialStateIsIdle) {
  EXPECT_EQ(algo_.getState(), State::IDLE);
  EXPECT_FALSE(algo_.isNavEnabled());
  EXPECT_EQ(algo_.getNavMode(), "stopped");
  EXPECT_FALSE(algo_.hasQueuedGoal());
}

TEST_F(MissionExecutiveAlgoTest, RegisterTarget) {
  TargetEntry entry{"t1", 10.0, 5.0, 0, 0, 3.0, false};
  auto res = algo_.setTarget(entry);
  EXPECT_TRUE(res.success);
}

TEST_F(MissionExecutiveAlgoTest, StartNavWithoutTargetFails) {
  auto res = algo_.startNav("", std::nullopt, false);
  EXPECT_FALSE(res.accepted);
  EXPECT_EQ(algo_.getState(), State::IDLE);
}

TEST_F(MissionExecutiveAlgoTest, StartNavWithUnknownTargetFails) {
  auto res = algo_.startNav("unknown", std::nullopt, false);
  EXPECT_FALSE(res.accepted);
  EXPECT_EQ(algo_.getState(), State::IDLE);
}

TEST_F(MissionExecutiveAlgoTest, StartNavValidTargetTransitionsToNavigating) {
  TargetEntry entry{"gnss1", 10.0, 10.0, 0, 0, 2.0, false};
  algo_.setTarget(entry);

  auto res = algo_.startNav("gnss1", std::nullopt, false);
  EXPECT_TRUE(res.accepted);
  EXPECT_TRUE(res.publish_goal);
  EXPECT_EQ(res.goal_to_publish.x, 10.0);
  EXPECT_EQ(res.goal_to_publish.y, 10.0);
  EXPECT_EQ(algo_.getState(), State::NAVIGATING);
  EXPECT_TRUE(algo_.isNavEnabled());
  EXPECT_EQ(algo_.getNavMode(), "autonomous");
}

TEST_F(MissionExecutiveAlgoTest, CrossTrackErrorReplan) {
  TargetEntry entry{"gnss1", 10.0, 0.0, 0, 0, 2.0, false};
  algo_.setTarget(entry);
  algo_.startNav("gnss1", std::nullopt, false);

  // Set a path from (0,0) to (10,0)
  std::vector<Pose2D> path = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
  algo_.updateGlobalPath(path);

  // Robot is at (5, 3), which is 3m away from the path y=0. Replan threshold is 2m.
  algo_.updateRobotPose({5.0, 3.0, 0.0});
  
  auto res = algo_.tick(0.0);
  EXPECT_TRUE(res.publish_goal);
  EXPECT_EQ(res.goal_to_publish.x, 10.0);
}

TEST_F(MissionExecutiveAlgoTest, ArrivalStateTransition) {
  TargetEntry entry{"gnss1", 10.0, 0.0, 0, 0, 2.0, false};
  algo_.setTarget(entry);
  algo_.startNav("gnss1", std::nullopt, false);

  // Robot moves within tolerance (1.5m < 2.0m)
  algo_.updateRobotPose({8.5, 0.0, 0.0});
  
  algo_.tick(0.0);
  EXPECT_EQ(algo_.getState(), State::ARRIVING);
}

TEST_F(MissionExecutiveAlgoTest, ArrivalHoldTimeAndStop) {
  TargetEntry entry{"gnss1", 10.0, 0.0, 0, 0, 2.0, false};
  algo_.setTarget(entry);
  algo_.startNav("gnss1", std::nullopt, false);
  
  algo_.updateRobotPose({9.0, 0.0, 0.0});
  algo_.tick(0.0); // Transitions to ARRIVING
  EXPECT_EQ(algo_.getState(), State::ARRIVING);

  // First IMU reading below threshold starts the timer
  algo_.updateImu(0.01, 10.0);
  algo_.tick(10.0);
  EXPECT_EQ(algo_.getState(), State::ARRIVING);

  // Tick before hold time expires
  algo_.updateImu(0.01, 10.5);
  algo_.tick(10.5);
  EXPECT_EQ(algo_.getState(), State::ARRIVING);

  // Tick after hold time expires
  algo_.updateImu(0.01, 11.1);
  auto tick_res = algo_.tick(11.1);
  EXPECT_EQ(algo_.getState(), State::STOPPED_AT_TARGET);
  EXPECT_TRUE(tick_res.action_finished);
  EXPECT_TRUE(tick_res.action_success);
}

TEST_F(MissionExecutiveAlgoTest, ArrivalWithSpiralCoverage) {
  // Target type 1 is ARUCO_POST, which triggers spiral coverage after stopping
  TargetEntry entry{"post1", 10.0, 0.0, 1, 0, 2.0, false};
  algo_.setTarget(entry);
  algo_.startNav("post1", std::nullopt, false);
  
  algo_.updateRobotPose({9.0, 0.0, 0.0});
  algo_.tick(0.0); // ARRIVING
  
  algo_.updateImu(0.01, 1.0);
  algo_.tick(1.0);
  algo_.updateImu(0.01, 2.1);
  auto tick_res = algo_.tick(2.1);
  
  // Since it's type 1, it transitions to SPIRAL_COVERAGE
  EXPECT_EQ(algo_.getState(), State::SPIRAL_COVERAGE);
  EXPECT_FALSE(tick_res.action_finished); // Action is not finished yet
  EXPECT_TRUE(tick_res.publish_goal); // Should publish first spiral waypoint
}

TEST_F(MissionExecutiveAlgoTest, SpiralCoverageCompletesOnDetection) {
  TargetEntry entry{"post1", 10.0, 0.0, 1, 0, 2.0, false};
  algo_.setTarget(entry);
  algo_.startNav("post1", std::nullopt, false);
  algo_.updateRobotPose({9.0, 0.0, 0.0});
  algo_.tick(0.0); // ARRIVING
  algo_.updateImu(0.01, 1.0);
  algo_.updateImu(0.01, 2.1);
  algo_.tick(2.1); // SPIRAL_COVERAGE

  EXPECT_EQ(algo_.getState(), State::SPIRAL_COVERAGE);
  
  // Detection occurs!
  algo_.onDetection();
  
  auto tick_res = algo_.tick(3.0);
  EXPECT_EQ(algo_.getState(), State::SPIRAL_DONE);
  EXPECT_TRUE(tick_res.action_finished);
  EXPECT_TRUE(tick_res.action_success);
}

TEST_F(MissionExecutiveAlgoTest, SpiralCoverageTimeout) {
  TargetEntry entry{"post1", 10.0, 0.0, 1, 0, 2.0, false};
  algo_.setTarget(entry);
  algo_.startNav("post1", std::nullopt, false);
  algo_.updateRobotPose({9.0, 0.0, 0.0});
  algo_.tick(0.0); // ARRIVING
  algo_.updateImu(0.01, 1.0);
  algo_.updateImu(0.01, 2.1);
  algo_.tick(2.1); // SPIRAL_COVERAGE

  // Jump time forward past the timeout
  auto tick_res = algo_.tick(2.1 + 61.0);
  
  EXPECT_EQ(algo_.getState(), State::SPIRAL_DONE);
  EXPECT_TRUE(tick_res.action_finished);
  EXPECT_TRUE(tick_res.action_success); // Timeout is considered completion
}

TEST_F(MissionExecutiveAlgoTest, ReturnFailsIfUnvisited) {
  TargetEntry entry{"post1", 10.0, 0.0, 1, 0, 2.0, false};
  algo_.setTarget(entry);
  
  // Attempt to return to an unvisited target
  auto res = algo_.startNav("post1", std::nullopt, true);
  EXPECT_FALSE(res.accepted);
}

TEST_F(MissionExecutiveAlgoTest, ReturnSucceedsIfVisited) {
  TargetEntry entry{"gnss1", 10.0, 0.0, 0, 0, 2.0, true};
  algo_.setTarget(entry);
  
  auto res = algo_.startNav("gnss1", std::nullopt, true);
  EXPECT_TRUE(res.accepted);
  EXPECT_EQ(algo_.getState(), State::RETURNING);
}

TEST_F(MissionExecutiveAlgoTest, AbortTransitionsToAborting) {
  TargetEntry entry{"gnss1", 10.0, 0.0, 0, 0, 2.0, false};
  algo_.setTarget(entry);
  algo_.startNav("gnss1", std::nullopt, false);
  EXPECT_EQ(algo_.getState(), State::NAVIGATING);

  auto res = algo_.abort();
  EXPECT_TRUE(res.success);
  EXPECT_EQ(algo_.getState(), State::ABORTING);

  // Ticking should complete the abort
  auto tick_res = algo_.tick(0.0);
  EXPECT_TRUE(tick_res.action_finished);
  EXPECT_FALSE(tick_res.action_success);
  EXPECT_EQ(algo_.getState(), State::IDLE);
}

TEST_F(MissionExecutiveAlgoTest, GoalQueuedDuringAbort) {
  TargetEntry entry1{"gnss1", 10.0, 0.0, 0, 0, 2.0, false};
  algo_.setTarget(entry1);
  algo_.startNav("gnss1", std::nullopt, false);
  
  algo_.abort();
  EXPECT_EQ(algo_.getState(), State::ABORTING);

  TargetEntry entry2{"gnss2", 20.0, 0.0, 0, 0, 2.0, false};
  algo_.setTarget(entry2);
  
  // Send new goal while aborting
  auto res = algo_.startNav("gnss2", std::nullopt, false);
  EXPECT_TRUE(res.accepted);
  EXPECT_TRUE(algo_.hasQueuedGoal());

  // Tick finishes abort, promotes queued goal
  auto tick_res = algo_.tick(0.0);
  EXPECT_TRUE(tick_res.action_finished);
  EXPECT_TRUE(tick_res.start_queued_goal);
  EXPECT_EQ(algo_.getState(), State::NAVIGATING);
  EXPECT_EQ(algo_.getActiveTarget()->id, "gnss2");
}

TEST_F(MissionExecutiveAlgoTest, PlanFailedTriggersAbort) {
  TargetEntry entry{"gnss1", 10.0, 0.0, 0, 0, 2.0, false};
  algo_.setTarget(entry);
  algo_.startNav("gnss1", std::nullopt, false);

  algo_.onPlanFailed();
  EXPECT_EQ(algo_.getState(), State::ABORTING);
}

TEST_F(MissionExecutiveAlgoTest, TeleopPreventsNav) {
  algo_.setTeleop(true);
  EXPECT_EQ(algo_.getState(), State::TELEOP);

  TargetEntry entry{"gnss1", 10.0, 0.0, 0, 0, 2.0, false};
  algo_.setTarget(entry);
  
  auto res = algo_.startNav("gnss1", std::nullopt, false);
  EXPECT_FALSE(res.accepted);

  algo_.setTeleop(false);
  EXPECT_EQ(algo_.getState(), State::IDLE);
}
