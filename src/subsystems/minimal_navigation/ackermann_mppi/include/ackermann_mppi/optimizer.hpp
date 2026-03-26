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

#ifndef ACKERMANN_MPPI__OPTIMIZER_HPP_
#define ACKERMANN_MPPI__OPTIMIZER_HPP_

#include <Eigen/Dense>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "ackermann_mppi/models/optimizer_settings.hpp"
#include "ackermann_mppi/motion_models.hpp"
#include "ackermann_mppi/critic_data.hpp"
#include "ackermann_mppi/models/state.hpp"
#include "ackermann_mppi/models/trajectories.hpp"
#include "ackermann_mppi/models/path.hpp"
#include "ackermann_mppi/tools/noise_generator.hpp"
#include "ackermann_mppi/tools/utils.hpp"

// Forward declarations for critic types
namespace mppi::critics
{
class CriticFunction;
class PathFollowCritic;
class PathAlignCritic;
class GoalCritic;
class ObstaclesCritic;
class ConstraintCritic;
}

namespace mppi
{

/**
 * @class mppi::Optimizer
 * @brief Ackermann-specific MPPI optimizer.
 *
 * Differences from the nav2_mppi_controller Optimizer:
 * - Initialized with rclcpp::Node::SharedPtr (not LifecycleNode)
 * - Critics are created directly (no pluginlib)
 * - No OptimalTrajectoryValidator — simple fallback on fail_flag
 * - Motion model hardcoded to AckermannMotionModel
 * - GoalChecker replaced with a distance threshold parameter
 * - Parameters read directly via node->declare_parameter / node->get_parameter
 */
class Optimizer
{
public:
  // Both constructor and destructor defined in optimizer.cpp so critic forward declarations are sufficient here
  Optimizer();
  ~Optimizer();

  /**
   * @brief Initialize the optimizer.
   * @param node ROS2 node for parameters, logging, and clock
   * @param name Parameter namespace prefix (e.g. "mppi")
   * @param costmap_ros Costmap for obstacle avoidance critics
   * @param tf_buffer TF buffer for robot pose lookups
   */
  void initialize(
    rclcpp::Node::SharedPtr node,
    const std::string & name,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer);

  void shutdown();

  /**
   * @brief Run MPPI to compute the next control command.
   * @param robot_pose Current robot pose (in costmap frame)
   * @param robot_speed Current robot velocity (from odometry)
   * @param plan Global path to follow
   * @param goal Goal pose (last pose of plan, or explicit goal)
   * @return [TwistStamped command, optimal trajectory as [time_steps x 3] (x, y, yaw)]
   * @throws std::runtime_error if all trajectories are in collision after retries
   */
  std::tuple<geometry_msgs::msg::TwistStamped, Eigen::ArrayXXf> evalControl(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    const nav_msgs::msg::Path & plan,
    const geometry_msgs::msg::Pose & goal);

  // --- Accessors for visualization / debugging ---

  models::Trajectories & getGeneratedTrajectories() { return generated_trajectories_; }
  Eigen::ArrayXXf getOptimizedTrajectory();
  const models::ControlSequence & getOptimalControlSequence() { return control_sequence_; }
  const Eigen::ArrayXf & getCosts() const { return costs_; }

  /**
   * @brief Per-critic cost breakdown from last evalControl call.
   * Each entry is (critic_name, cost_delta_array[batch_size]).
   */
  const std::vector<std::pair<std::string, Eigen::ArrayXf>> & getCriticCosts() const
  {
    return critic_costs_;
  }

  const std::vector<bool> & getCollisionFlags() const
  {
    return critics_data_.trajectories_in_collision;
  }

  /**
   * @brief Scale max speeds by a ratio (e.g. from a speed zone costmap filter).
   * @param speed_limit Absolute speed OR percentage of max speed
   * @param percentage True if speed_limit is a percentage [0–100]
   */
  void setSpeedLimit(double speed_limit, bool percentage);

  void reset(bool reset_dynamic_speed_limits = true);

  bool isSpeedLimitActive() const;

  const models::OptimizerSettings & getSettings() const { return settings_; }

protected:
  void optimize();
  void evalTrajectoriesScores();

  void prepare(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    const nav_msgs::msg::Path & plan,
    const geometry_msgs::msg::Pose & goal);

  void getParams();
  void loadCritics();

  void shiftControlSequence();
  void generateNoisedTrajectories();
  void applyControlSequenceConstraints();
  void updateStateVelocities(models::State & state) const;
  void updateInitialStateVelocities(models::State & state) const;
  void propagateStateVelocitiesFromInitials(models::State & state) const;

  void integrateStateVelocities(
    models::Trajectories & trajectories,
    const models::State & state) const;

  void integrateStateVelocities(
    Eigen::Array<float, Eigen::Dynamic, 3> & trajectory,
    const Eigen::ArrayXXf & sequence) const;

  void updateControlSequence();

  geometry_msgs::msg::TwistStamped
  getControlFromSequenceAsTwist(const builtin_interfaces::msg::Time & stamp);

  bool fallback(bool fail);

  size_t fallback_counter_{0};  // member — not static, so reset() clears it correctly

  template<typename T>
  void declareParam(const std::string & full_name, T default_value)
  {
    if (!node_->has_parameter(full_name)) {
      node_->declare_parameter(full_name, default_value);
    }
  }

  auto getParamGetter(const std::string & ns)
  {
    return [this, ns](auto & setting, const std::string & name, auto default_value) {
             std::string full_name = ns.empty() ? name : ns + "." + name;
             declareParam(full_name, default_value);
             node_->get_parameter(full_name, setting);
           };
  }

protected:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string name_;
  std::string base_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  std::shared_ptr<AckermannMotionModel> motion_model_;

  // Critics owned directly (no pluginlib)
  std::vector<std::unique_ptr<critics::CriticFunction>> critics_;
  std::vector<std::pair<std::string, Eigen::ArrayXf>> critic_costs_;

  NoiseGenerator noise_generator_;

  models::OptimizerSettings settings_;
  float goal_dist_tolerance_{0.25f};

  models::State state_;
  models::ControlSequence control_sequence_;
  std::array<mppi::models::Control, 4> control_history_;
  models::Trajectories generated_trajectories_;
  models::Path path_;
  geometry_msgs::msg::Pose goal_;
  Eigen::ArrayXf costs_;

  CriticData critics_data_{
    state_,
    generated_trajectories_,
    path_,
    goal_,
    costs_,
    settings_.model_dt,
    goal_dist_tolerance_};

  rclcpp::Logger logger_{rclcpp::get_logger("AckermannMPPI")};
  geometry_msgs::msg::Twist last_command_vel_;
};

}  // namespace mppi

#endif  // ACKERMANN_MPPI__OPTIMIZER_HPP_
