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

#ifndef ACKERMANN_MPPI__TOOLS__NOISE_GENERATOR_HPP_
#define ACKERMANN_MPPI__TOOLS__NOISE_GENERATOR_HPP_

#include <Eigen/Dense>

#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <random>

#include "ackermann_mppi/models/optimizer_settings.hpp"
#include "ackermann_mppi/models/control_sequence.hpp"
#include "ackermann_mppi/models/state.hpp"

namespace mppi
{

/**
 * @class mppi::NoiseGenerator
 * @brief Generates Gaussian-perturbed control sequences for MPPI sampling.
 *
 * When regenerate_noises=true (set via optimizer param), noise generation runs in a
 * background thread that pre-generates noise for the *next* iteration while the
 * current one scores trajectories. This hides latency on slower hardware.
 *
 * When regenerate_noises=false (default), noise is generated synchronously each call.
 */
class NoiseGenerator
{
public:
  NoiseGenerator() = default;

  /**
   * @brief Initialize noise generator.
   * @param settings Optimizer settings (batch_size, time_steps, sampling_std)
   * @param regenerate_noises If true, run noise generation in background thread
   */
  void initialize(
    mppi::models::OptimizerSettings & settings,
    bool regenerate_noises = false);

  void shutdown();

  /**
   * @brief Signal the background thread to generate next iteration's noises.
   * No-op when regenerate_noises=false.
   */
  void generateNextNoises();

  /**
   * @brief Apply noises to the current optimal control sequence to get noised controls.
   * Stores in state.cvx / state.cwz for use by the motion model predict().
   */
  void setNoisedControls(
    models::State & state,
    const models::ControlSequence & control_sequence);

  void reset(mppi::models::OptimizerSettings & settings);

protected:
  void noiseThread();
  void generateNoisedControls();

  Eigen::ArrayXXf noises_vx_;
  Eigen::ArrayXXf noises_wz_;

  std::default_random_engine generator_;
  std::normal_distribution<float> ndistribution_vx_;
  std::normal_distribution<float> ndistribution_wz_;

  mppi::models::OptimizerSettings settings_;
  bool regenerate_noises_{false};

  std::thread noise_thread_;
  std::condition_variable noise_cond_;
  std::mutex noise_lock_;
  bool active_{false}, ready_{false};
};

}  // namespace mppi

#endif  // ACKERMANN_MPPI__TOOLS__NOISE_GENERATOR_HPP_
