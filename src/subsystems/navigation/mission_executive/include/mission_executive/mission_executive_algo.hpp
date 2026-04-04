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

#include <string>
#include <vector>
#include <optional>
#include <unordered_map>
#include <cstdint>

namespace mission_executive {

enum class State {
  IDLE,
  NAVIGATING,
  ARRIVING,
  STOPPED_AT_TARGET,
  SPIRAL_COVERAGE,
  SPIRAL_DONE,
  ABORTING,
  RETURNING,
  STOPPED_AT_RETURN,
  TELEOP
};

std::string stateToStr(State s);

struct Pose2D {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct TargetEntry {
  std::string id;
  double x_m{0.0};
  double y_m{0.0};
  uint8_t target_type{0};
  uint8_t goal_source{0}; 
  double tolerance_m{3.0};
  bool visited{false};
};

struct MissionParams {
  double stop_angular_vel_threshold = 0.05;
  double arrival_hold_time = 1.0;
  double replan_distance_m = 3.0;
  double spiral_timeout_s = 120.0;
  double spiral_radius_m = 15.0;
  double spiral_spacing_m = 2.0;
  double spiral_angular_step = 0.5;
  double spiral_waypoint_tolerance_m = 2.0;
};

struct CommandResult {
  bool success;
  std::string message;
};

struct StartNavResult {
  bool accepted = false;
  std::string message;
  bool preempted_old = false;
  bool publish_goal = false;
  Pose2D goal_to_publish;
};

struct TickResult {
  bool publish_goal = false;
  Pose2D goal_to_publish;

  bool action_finished = false;
  bool action_success = false;
  std::string action_message;

  bool start_queued_goal = false;
};

class MissionExecutiveAlgo {
public:
  MissionExecutiveAlgo() = default;

  /*
   * Sets the configuration parameters for the mission executive.
   * Param: p - The new mission parameters to apply.
   */
  void setParams(const MissionParams& p) { params_ = p; }

  // Inputs

  /*
   * Updates the robot's current estimated pose.
   * Param: pose - The latest 2D pose (x, y, yaw).
   */
  void updateRobotPose(const Pose2D& pose) { robot_pose_ = pose; }
  
  /*
   * Updates the planned global path for cross-track error calculations.
   * Param: path - A sequence of 2D poses representing the path.
   */
  void updateGlobalPath(const std::vector<Pose2D>& path) { global_path_ = path; }
  
  /*
   * Updates the current IMU angular velocity, used to detect if the robot has stopped.
   * Param: angular_vel_mag - Magnitude of angular velocity.
   * Param: current_time_s - Current time in seconds.
   */
  void updateImu(double angular_vel_mag, double current_time_s);
  
  /*
   * Records the latest planner event from the navigation stack.
   * Param: event - The event code (e.g., plan failed).
   */
  void updatePlannerEvent(uint8_t event) { last_planner_event_ = event; }
  
  /*
   * Handles a failure in the global planner by aborting the current navigation.
   */
  void onPlanFailed();
  
  /*
   * Handles a successful visual detection, usually ending a spiral search.
   */
  void onDetection();

  // Commands
  
  /*
   * Adds or updates a target in the internal target registry.
   * Param: entry - The target information to store.
   * Returns: CommandResult indicating success or failure.
   */
  CommandResult setTarget(const TargetEntry& entry);
  
  /*
   * Initiates navigation to a specific target or an inline provided target.
   * Param: target_id - The ID of a registered target (can be empty).
   * Param: inline_target - A target provided directly (used if target_id is empty).
   * Param: is_return - True if this is a return trip to a previously visited target.
   * Returns: StartNavResult detailing if the command was accepted and any goals to publish.
   */
  StartNavResult startNav(const std::string& target_id, const std::optional<TargetEntry>& inline_target, bool is_return);
  
  /*
   * Aborts the current navigation or autonomous action.
   * Returns: CommandResult indicating if the abort was successful.
   */
  CommandResult abort();

  /*
   * Toggles the teleoperation state, preventing autonomous navigation when enabled.
   * Param: enable - True to enable teleop, false to return to idle.
   * Returns: CommandResult indicating success.
   */
  CommandResult setTeleop(bool enable);

  /*
   * Cancels the active navigation and resets the state to IDLE.
   * Returns: CommandResult indicating success.
   */
  CommandResult cancelNav(); 
  
  /*
   * Main update step for the state machine, evaluated periodically.
   * Param: current_time_s - Current system time in seconds.
   * Returns: TickResult with actions the node should perform (e.g., publishing goals).
   */
  TickResult tick(double current_time_s);

  // Getters
  State getState() const { return state_; }
  bool isNavEnabled() const {
    return state_ == State::NAVIGATING || state_ == State::ARRIVING ||
           state_ == State::RETURNING || state_ == State::SPIRAL_COVERAGE;
  }
  std::string getNavMode() const;
  std::optional<TargetEntry> getActiveTarget() const { return active_target_; }
  double getDistToGoal() const;
  double getCrossTrackError() const;
  double getHeadingError() const;
  double getImuAngularVel() const { return imu_angular_vel_; }
  bool isReturn() const { return is_return_; }
  uint8_t getLastPlannerEvent() const { return last_planner_event_; }
  bool hasQueuedGoal() const { return queued_active_; }

private:
  void transition(State new_state);
  void startSpiralCoverage(double current_time_s);
  std::vector<Pose2D> generateSpiralWaypoints(const Pose2D& start);

  State state_{State::IDLE};
  MissionParams params_;

  std::unordered_map<std::string, TargetEntry> target_registry_;
  std::optional<TargetEntry> active_target_;
  bool is_return_{false};

  bool queued_active_{false};
  TargetEntry queued_entry_;
  bool queued_is_return_{false};

  std::optional<Pose2D> robot_pose_;
  std::optional<std::vector<Pose2D>> global_path_;
  
  double imu_angular_vel_{0.0};
  double low_speed_start_s_{0.0};
  bool low_speed_tracking_{false};
  uint8_t last_planner_event_{0};

  std::vector<Pose2D> spiral_waypoints_;
  size_t spiral_waypoint_idx_{0};
  double spiral_start_time_s_{0.0};
  bool detection_received_{false};
};

} // namespace mission_executive
