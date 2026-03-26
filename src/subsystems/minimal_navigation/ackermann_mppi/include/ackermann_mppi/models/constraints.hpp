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

#ifndef ACKERMANN_MPPI__MODELS__CONSTRAINTS_HPP_
#define ACKERMANN_MPPI__MODELS__CONSTRAINTS_HPP_

namespace mppi::models
{

/**
 * @struct mppi::models::ControlConstraints
 * @brief Constraints on control (Ackermann: no vy/ay terms)
 */
struct ControlConstraints
{
  float vx_max;
  float vx_min;
  float wz;
  float ax_max;
  float ax_min;
  float az_max;
};

/**
 * @struct mppi::models::SamplingStd
 * @brief Noise standard deviations for MPPI sampling (Ackermann: vx and wz only)
 */
struct SamplingStd
{
  float vx;
  float wz;
};

}  // namespace mppi::models

#endif  // ACKERMANN_MPPI__MODELS__CONSTRAINTS_HPP_
