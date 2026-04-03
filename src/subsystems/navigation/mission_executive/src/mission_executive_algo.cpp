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

#include "mission_executive/mission_executive_algo.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

namespace mission_executive {

std::string stateToStr(State s) {
  switch (s) {
    case State::IDLE:              return "IDLE";
    case State::NAVIGATING:        return "NAVIGATING";
    case State::ARRIVING:          return "ARRIVING";
    case State::STOPPED_AT_TARGET: return "STOPPED_AT_TARGET";
    case State::SPIRAL_COVERAGE:   return "SPIRAL_COVERAGE";
    case State::SPIRAL_DONE:       return "SPIRAL_DONE";
    case State::ABORTING:          return "ABORTING";
    case State::RETURNING:         return "RETURNING";
    case State::STOPPED_AT_RETURN: return "STOPPED_AT_RETURN";
    case State::TELEOP:            return "TELEOP";
    default:                       return "UNKNOWN";
  }
}

void MissionExecutiveAlgo::transition(State new_state) {
  state_ = new_state;
  if (state_ == State::TELEOP || !isNavEnabled()) {
    low_speed_tracking_ = false;
  }
}

std::string MissionExecutiveAlgo::getNavMode() const {
  switch (state_) {
    case State::TELEOP: return "teleop";
    case State::NAVIGATING:
    case State::ARRIVING:
    case State::RETURNING:
    case State::SPIRAL_COVERAGE: return "autonomous";
    default: return "stopped";
  }
}

double MissionExecutiveAlgo::getHeadingError() const {
  if (!robot_pose_.has_value() || !active_target_.has_value()) return 0.0;
  const double dx = active_target_->x_m - robot_pose_->x;
  const double dy = active_target_->y_m - robot_pose_->y;
  const double bearing = std::atan2(dy, dx);
  double err = bearing - robot_pose_->yaw;
  while (err >  M_PI) err -= 2.0 * M_PI;
  while (err < -M_PI) err += 2.0 * M_PI;
  return err;
}

double MissionExecutiveAlgo::getDistToGoal() const {
  if (!robot_pose_.has_value() || !active_target_.has_value()) return -1.0;
  return std::hypot(robot_pose_->x - active_target_->x_m, robot_pose_->y - active_target_->y_m);
}

double MissionExecutiveAlgo::getCrossTrackError() const {
  if (!robot_pose_.has_value() || !global_path_.has_value()) return -1.0;
  const auto & poses = global_path_.value();
  if (poses.size() < 2) return -1.0;

  const double rx = robot_pose_->x;
  const double ry = robot_pose_->y;
  double min_dist = std::numeric_limits<double>::max();

  for (size_t i = 0; i < poses.size() - 1; ++i) {
    const double ax = poses[i].x,  ay = poses[i].y;
    const double bx = poses[i+1].x, by = poses[i+1].y;
    const double dx = bx - ax, dy = by - ay;
    const double len2 = dx*dx + dy*dy;
    double dist;
    if (len2 < 1e-10) {
      dist = std::hypot(rx - ax, ry - ay);
    } else {
      const double t = std::clamp(((rx-ax)*dx + (ry-ay)*dy) / len2, 0.0, 1.0);
      dist = std::hypot(rx - (ax + t*dx), ry - (ay + t*dy));
    }
    min_dist = std::min(min_dist, dist);
  }
  return min_dist;
}

void MissionExecutiveAlgo::updateImu(double angular_vel_mag, double current_time_s) {
  imu_angular_vel_ = angular_vel_mag;
  if (state_ == State::ARRIVING) {
    if (imu_angular_vel_ < params_.stop_angular_vel_threshold) {
      if (!low_speed_tracking_) {
        low_speed_start_s_ = current_time_s;
        low_speed_tracking_ = true;
      } else {
        if (current_time_s - low_speed_start_s_ >= params_.arrival_hold_time) {
          low_speed_tracking_ = false;
          if (is_return_) {
            transition(State::STOPPED_AT_RETURN);
          } else {
            if (active_target_.has_value()) {
              auto it = target_registry_.find(active_target_->id);
              if (it != target_registry_.end()) {
                it->second.visited = true;
              }
            }
            transition(State::STOPPED_AT_TARGET);
          }
        }
      }
    } else {
      low_speed_tracking_ = false;
    }
  }
}

void MissionExecutiveAlgo::onPlanFailed() {
  if (state_ == State::NAVIGATING || state_ == State::RETURNING) {
    transition(State::ABORTING);
  }
}

void MissionExecutiveAlgo::onDetection() {
  if (state_ == State::SPIRAL_COVERAGE) {
    detection_received_ = true;
    transition(State::SPIRAL_DONE);
  }
}

CommandResult MissionExecutiveAlgo::setTarget(const TargetEntry& entry) {
  target_registry_[entry.id] = entry;
  return {true, "Target '" + entry.id + "' registered"};
}

StartNavResult MissionExecutiveAlgo::startNav(const std::string& target_id, const std::optional<TargetEntry>& inline_target, bool is_return) {
  StartNavResult res;
  
  TargetEntry entry;
  if (!target_id.empty()) {
    auto it = target_registry_.find(target_id);
    if (it == target_registry_.end()) {
      res.accepted = false;
      res.message = "Unknown target_id: " + target_id;
      return res;
    }
    if (is_return && !it->second.visited) {
      res.accepted = false;
      res.message = "Target not yet visited — cannot RETURN";
      return res;
    }
    entry = it->second;
  } else if (inline_target.has_value()) {
    entry = inline_target.value();
  } else {
    res.accepted = false;
    res.message = "No target provided";
    return res;
  }

  if (state_ == State::TELEOP) {
    res.accepted = false;
    res.message = "Cannot navigate — currently in TELEOP mode";
    return res;
  }

  if (state_ == State::ABORTING) {
    if (queued_active_) {
      res.preempted_old = true;
    }
    queued_active_ = true;
    queued_entry_ = entry;
    queued_is_return_ = is_return;
    res.accepted = true;
    res.message = "Goal queued — currently ABORTING";
    return res;
  }

  bool is_active_nav = (state_ != State::IDLE && state_ != State::STOPPED_AT_TARGET && state_ != State::STOPPED_AT_RETURN && state_ != State::SPIRAL_DONE);
  if (is_active_nav) {
    res.preempted_old = true;
  }

  active_target_ = entry;
  is_return_ = is_return;
  transition(is_return ? State::RETURNING : State::NAVIGATING);
  
  res.accepted = true;
  res.publish_goal = true;
  res.goal_to_publish = {entry.x_m, entry.y_m, 0.0};
  return res;
}

CommandResult MissionExecutiveAlgo::abort() {
  if (state_ == State::IDLE || state_ == State::TELEOP ||
      state_ == State::ABORTING ||
      state_ == State::STOPPED_AT_TARGET || state_ == State::STOPPED_AT_RETURN ||
      state_ == State::SPIRAL_DONE) {
    return {false, "Nothing to abort in state " + stateToStr(state_)};
  }
  transition(State::ABORTING);
  return {true, "Aborting"};
}

CommandResult MissionExecutiveAlgo::setTeleop(bool enable) {
  if (enable) {
    transition(State::TELEOP);
    return {true, "Teleop ON"};
  } else {
    if (state_ == State::TELEOP) {
      transition(State::IDLE);
      return {true, "Teleop OFF"};
    } else {
      return {false, "Not in TELEOP state"};
    }
  }
}

CommandResult MissionExecutiveAlgo::cancelNav() {
  transition(State::IDLE);
  return {true, "Cancelled"};
}

std::vector<Pose2D> MissionExecutiveAlgo::generateSpiralWaypoints(const Pose2D& start) {
  std::vector<Pose2D> waypoints;
  const double a = params_.spiral_spacing_m / (2.0 * M_PI);
  if (a <= 0.0) return waypoints;

  const double max_angle       = params_.spiral_radius_m / a;
  const double min_start_r     = 1.5;
  const double yaw0            = start.yaw;

  double theta = 0.0;
  while (theta <= max_angle) {
    const double r = min_start_r + a * theta;
    if (r > params_.spiral_radius_m) break;

    const double x = start.x + r * std::cos(yaw0 + theta + M_PI_2);
    const double y = start.y + r * std::sin(yaw0 + theta + M_PI_2);

    waypoints.push_back({x, y, 0.0});
    theta += params_.spiral_angular_step;
  }
  return waypoints;
}

void MissionExecutiveAlgo::startSpiralCoverage(double current_time_s) {
  if (!robot_pose_.has_value()) {
    transition(State::SPIRAL_DONE);
    return;
  }

  spiral_waypoints_ = generateSpiralWaypoints(robot_pose_.value());
  spiral_waypoint_idx_ = 0;
  spiral_start_time_s_ = current_time_s;
  detection_received_ = false;

  if (spiral_waypoints_.empty()) {
    transition(State::SPIRAL_DONE);
  }
}

TickResult MissionExecutiveAlgo::tick(double current_time_s) {
  TickResult res;

  // 1. checkArrival
  if ((state_ == State::NAVIGATING || state_ == State::RETURNING) && active_target_.has_value()) {
    double d = getDistToGoal();
    if (d >= 0.0 && d < active_target_->tolerance_m) {
      transition(State::ARRIVING);
    }
  }

  // 2. checkCrossTrackError
  if ((state_ == State::NAVIGATING || state_ == State::RETURNING) && active_target_.has_value()) {
    double xte = getCrossTrackError();
    if (xte >= 0.0 && xte > params_.replan_distance_m) {
      res.publish_goal = true;
      res.goal_to_publish = {active_target_->x_m, active_target_->y_m, 0.0};
    }
  }

  // 3. checkSpiralProgress
  if (state_ == State::SPIRAL_COVERAGE) {
    if (current_time_s - spiral_start_time_s_ >= params_.spiral_timeout_s) {
      transition(State::SPIRAL_DONE);
    } else if (robot_pose_.has_value() && !spiral_waypoints_.empty()) {
      const auto & wp = spiral_waypoints_[spiral_waypoint_idx_];
      const double dist = std::hypot(robot_pose_->x - wp.x, robot_pose_->y - wp.y);
      if (dist < params_.spiral_waypoint_tolerance_m) {
        ++spiral_waypoint_idx_;
        if (spiral_waypoint_idx_ >= spiral_waypoints_.size()) {
          transition(State::SPIRAL_DONE);
        } else {
          res.publish_goal = true;
          res.goal_to_publish = spiral_waypoints_[spiral_waypoint_idx_];
        }
      }
    }
  }

  // 4. Action result resolution
  if (state_ == State::STOPPED_AT_TARGET) {
    // ARUCO_POST = 1, OBJECT = 2
    const bool needs_spiral = active_target_.has_value() &&
      (active_target_->target_type == 1 || active_target_->target_type == 2);
    
    if (needs_spiral) {
      transition(State::SPIRAL_COVERAGE);
      startSpiralCoverage(current_time_s);
      if (state_ == State::SPIRAL_COVERAGE && !spiral_waypoints_.empty()) {
        res.publish_goal = true;
        res.goal_to_publish = spiral_waypoints_[0];
      }
    } else {
      res.action_finished = true;
      res.action_success = true;
      res.action_message = "Arrived at target";
    }
  } else if (state_ == State::SPIRAL_DONE) {
    res.action_finished = true;
    res.action_success = true;
    res.action_message = detection_received_ ? "Detection found during spiral coverage" : "Spiral coverage complete (timeout or all waypoints visited)";
  } else if (state_ == State::STOPPED_AT_RETURN) {
    res.action_finished = true;
    res.action_success = true;
    res.action_message = "Arrived at target";
  } else if (state_ == State::ABORTING) {
    res.action_finished = true;
    res.action_success = false;
    res.action_message = "Aborted";

    if (queued_active_) {
      active_target_ = queued_entry_;
      is_return_ = queued_is_return_;
      queued_active_ = false;
      transition(is_return_ ? State::RETURNING : State::NAVIGATING);
      res.start_queued_goal = true;
      res.publish_goal = true;
      res.goal_to_publish = {active_target_->x_m, active_target_->y_m, 0.0};
    } else {
      transition(State::IDLE);
    }
  }

  return res;
}

} // namespace mission_executive
