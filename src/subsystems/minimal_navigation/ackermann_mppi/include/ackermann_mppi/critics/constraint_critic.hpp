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

#ifndef ACKERMANN_MPPI__CRITICS__CONSTRAINT_CRITIC_HPP_
#define ACKERMANN_MPPI__CRITICS__CONSTRAINT_CRITIC_HPP_

#include "ackermann_mppi/critic_function.hpp"

namespace mppi::critics
{

/**
 * @class ConstraintCritic (Ackermann-only)
 * @brief Penalizes trajectories that violate velocity and turning radius constraints.
 *
 * For Ackermann: penalizes vx outside [vx_min, vx_max] and steering ratios
 * (|vx|/|wz|) that would require a tighter turn than min_turning_r.
 */
class ConstraintCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;

protected:
  unsigned int power_{0};
  float weight_{0};
  // min_turning_r is vehicle geometry — fixed. Velocity limits are read from
  // data.motion_model->getConstraints() in score() so speed-limit changes take effect.
  float min_turning_r_{0.2f};
};

}  // namespace mppi::critics

#endif  // ACKERMANN_MPPI__CRITICS__CONSTRAINT_CRITIC_HPP_
