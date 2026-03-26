// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#ifndef ACKERMANN_MPPI__CRITIC_DATA_HPP_
#define ACKERMANN_MPPI__CRITIC_DATA_HPP_

#include <Eigen/Dense>

#include <memory>
#include <optional>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ackermann_mppi/models/state.hpp"
#include "ackermann_mppi/models/trajectories.hpp"
#include "ackermann_mppi/models/path.hpp"
#include "ackermann_mppi/motion_models.hpp"

namespace mppi
{

/**
 * @struct mppi::CriticData
 * @brief Data passed to critics for scoring. GoalChecker replaced with a simple
 * distance tolerance float — the optimizer sets this from its parameters.
 *
 * Has an explicit constructor to prevent silent wrong-field binding when using
 * aggregate initialization in C++17 (designated initializers require C++20).
 */
struct CriticData
{
  const models::State & state;
  const models::Trajectories & trajectories;
  const models::Path & path;
  const geometry_msgs::msg::Pose & goal;

  Eigen::ArrayXf & costs;
  float & model_dt;

  bool fail_flag{false};

  // Replaces nav2_core::GoalChecker — set by Optimizer::prepare() from settings
  float goal_dist_tolerance{0.25f};

  std::shared_ptr<MotionModel> motion_model;
  std::optional<std::vector<bool>> path_pts_valid;
  std::optional<size_t> furthest_reached_path_point;
  std::vector<bool> trajectories_in_collision;

  // Explicit constructor so call sites use named parameters, preventing silent
  // mis-binding if fields are reordered.
  CriticData(
    const models::State & state_in,
    const models::Trajectories & trajectories_in,
    const models::Path & path_in,
    const geometry_msgs::msg::Pose & goal_in,
    Eigen::ArrayXf & costs_in,
    float & model_dt_in,
    float goal_dist_tolerance_in = 0.25f,
    std::shared_ptr<MotionModel> motion_model_in = nullptr)
  : state(state_in),
    trajectories(trajectories_in),
    path(path_in),
    goal(goal_in),
    costs(costs_in),
    model_dt(model_dt_in),
    fail_flag(false),
    goal_dist_tolerance(goal_dist_tolerance_in),
    motion_model(std::move(motion_model_in)) {}
};

}  // namespace mppi

#endif  // ACKERMANN_MPPI__CRITIC_DATA_HPP_
