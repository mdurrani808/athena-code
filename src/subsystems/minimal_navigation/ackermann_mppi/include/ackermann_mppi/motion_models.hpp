// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2025 Open Navigation LLC
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

#ifndef ACKERMANN_MPPI__MOTION_MODELS_HPP_
#define ACKERMANN_MPPI__MOTION_MODELS_HPP_

#include <Eigen/Dense>
#include <cstdint>
#include <algorithm>

#include "ackermann_mppi/models/control_sequence.hpp"
#include "ackermann_mppi/models/state.hpp"
#include "ackermann_mppi/models/constraints.hpp"

namespace mppi
{

// Forward declaration used in utils.hpp to avoid circular include
namespace utils
{
float clamp(const float lower_bound, const float upper_bound, const float input);
}

/**
 * @class mppi::MotionModel
 * @brief Abstract motion model base class
 */
class MotionModel
{
public:
  MotionModel() = default;
  virtual ~MotionModel() = default;

  void initialize(const models::ControlConstraints & control_constraints, float model_dt)
  {
    control_constraints_ = control_constraints;
    model_dt_ = model_dt;
  }

  /**
   * @brief Propagate velocities forward one step using acceleration constraints.
   * Batch operation: state matrices are [batch_size x time_steps].
   */
  virtual void predict(models::State & state)
  {
    float max_delta_vx = model_dt_ * control_constraints_.ax_max;
    float min_delta_vx = model_dt_ * control_constraints_.ax_min;
    float max_delta_wz = model_dt_ * control_constraints_.az_max;

    unsigned int n_cols = state.vx.cols();
    for (unsigned int i = 1; i < n_cols; i++) {
      auto lower_bound_vx = (state.vx.col(i - 1) > 0).select(
        state.vx.col(i - 1) + min_delta_vx,
        state.vx.col(i - 1) - max_delta_vx);
      auto upper_bound_vx = (state.vx.col(i - 1) > 0).select(
        state.vx.col(i - 1) + max_delta_vx,
        state.vx.col(i - 1) - min_delta_vx);

      state.cvx.col(i - 1) = state.cvx.col(i - 1)
        .cwiseMax(lower_bound_vx)
        .cwiseMin(upper_bound_vx);
      state.vx.col(i) = state.cvx.col(i - 1);

      state.cwz.col(i - 1) = state.cwz.col(i - 1)
        .cwiseMax(state.wz.col(i - 1) - max_delta_wz)
        .cwiseMin(state.wz.col(i - 1) + max_delta_wz);
      state.wz.col(i) = state.cwz.col(i - 1);
    }
  }

  virtual bool isHolonomic() = 0;

  /**
   * @brief Apply model-specific hard constraints to the optimal control sequence.
   * Called after softmax weighting, before returning the command.
   */
  virtual void applyConstraints(models::ControlSequence & /*control_sequence*/) {}

  /**
   * @brief Return the currently active constraints (may differ from base if speed limit is set).
   * Critics should read from this instead of caching their own copies.
   */
  const models::ControlConstraints & getConstraints() const { return control_constraints_; }

protected:
  float model_dt_{0.0};
  models::ControlConstraints control_constraints_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
};

/**
 * @class mppi::AckermannMotionModel
 * @brief Ackermann steering motion model.
 *
 * Key constraint: angular velocity is bounded by |wz| <= |vx| / min_turning_r
 * This prevents commanding steering angles that are physically impossible at a given speed.
 */
class AckermannMotionModel : public MotionModel
{
public:
  explicit AckermannMotionModel(float min_turning_r = 0.2f)
  : min_turning_r_(min_turning_r) {}

  bool isHolonomic() override { return false; }

  void applyConstraints(models::ControlSequence & control_sequence) override
  {
    const auto wz_constrained = control_sequence.vx.abs() / min_turning_r_;
    control_sequence.wz = control_sequence.wz
      .max(-wz_constrained)
      .min(wz_constrained);
  }

  float getMinTurningRadius() const { return min_turning_r_; }

private:
  float min_turning_r_{0.2f};
};

}  // namespace mppi

#endif  // ACKERMANN_MPPI__MOTION_MODELS_HPP_
