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

#include "ackermann_mppi/critics/constraint_critic.hpp"

namespace mppi::critics
{

void ConstraintCritic::initialize()
{
  auto getParam = getParamGetter(name_);
  auto getParentParam = getParamGetter(parent_name_);

  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 4.0f);

  // min_turning_r is vehicle geometry — fixed at init.
  // vx_min/vx_max are read live from data.motion_model in score() so speed limits take effect.
  getParentParam(min_turning_r_, "AckermannConstraints.min_turning_r", 0.2f);

  RCLCPP_INFO(
    logger_, "ConstraintCritic (Ackermann) instantiated with %d power and %f weight.",
    power_, weight_);
}

void ConstraintCritic::score(CriticData & data)
{
  if (!enabled_) {return;}

  // Read velocity limits live so that setSpeedLimit() changes take effect immediately.
  const auto & c = data.motion_model->getConstraints();
  const float max_vel = c.vx_max;
  // Preserve original sign convention: vx_min is typically negative for reverse.
  const float min_vel = (c.vx_min > 0.0f ? 1.0f : -1.0f) * std::abs(c.vx_min);

  auto & vx = data.state.vx;
  auto & wz = data.state.wz;

  // Penalize steering ratio violations: |wz| <= |vx| / min_turning_r
  constexpr float kEpsilon = 1e-6f;
  auto wz_safe = wz.abs().max(kEpsilon);
  auto out_of_turning_rad_motion = (min_turning_r_ - (vx.abs() / wz_safe)).max(0.0f);

  if (power_ > 1u) {
    data.costs += ((((vx - max_vel).max(0.0f) + (min_vel - vx).max(0.0f) +
      out_of_turning_rad_motion) * data.model_dt).rowwise().sum().eval() *
      weight_).pow(power_).eval();
  } else {
    data.costs += ((((vx - max_vel).max(0.0f) + (min_vel - vx).max(0.0f) +
      out_of_turning_rad_motion) * data.model_dt).rowwise().sum().eval() * weight_).eval();
  }
}

}  // namespace mppi::critics
