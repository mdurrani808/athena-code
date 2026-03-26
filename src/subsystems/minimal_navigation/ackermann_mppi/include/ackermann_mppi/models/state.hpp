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

#ifndef ACKERMANN_MPPI__MODELS__STATE_HPP_
#define ACKERMANN_MPPI__MODELS__STATE_HPP_

#include <Eigen/Dense>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>


namespace mppi::models
{

/**
 * @struct mppi::models::State
 * @brief State information: velocities, controls, poses, speed
 */
struct State
{
  // Propagated velocities: [batch_size x time_steps]
  Eigen::ArrayXXf vx;   // longitudinal velocity
  Eigen::ArrayXXf wz;   // angular velocity (vy is always zero for Ackermann)

  // Noised controls: [batch_size x time_steps]
  Eigen::ArrayXXf cvx;
  Eigen::ArrayXXf cwz;

  geometry_msgs::msg::PoseStamped pose;
  geometry_msgs::msg::Twist speed;
  float local_path_length;

  void reset(unsigned int batch_size, unsigned int time_steps)
  {
    vx.setZero(batch_size, time_steps);
    wz.setZero(batch_size, time_steps);
    cvx.setZero(batch_size, time_steps);
    cwz.setZero(batch_size, time_steps);
  }
};
}  // namespace mppi::models

#endif  // ACKERMANN_MPPI__MODELS__STATE_HPP_
