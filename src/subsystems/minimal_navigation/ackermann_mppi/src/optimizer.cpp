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

#include "ackermann_mppi/optimizer.hpp"

#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <cmath>

#include "ackermann_mppi/critics/path_follow_critic.hpp"
#include "ackermann_mppi/critics/path_align_critic.hpp"
#include "ackermann_mppi/critics/goal_critic.hpp"
#include "ackermann_mppi/critics/obstacles_critic.hpp"
#include "ackermann_mppi/critics/constraint_critic.hpp"

namespace mppi
{

Optimizer::Optimizer() = default;

Optimizer::~Optimizer()
{
  shutdown();
}

void Optimizer::initialize(
  rclcpp::Node::SharedPtr node,
  const std::string & name,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer)
{
  node_ = node;
  name_ = name;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_buffer_ = tf_buffer;
  logger_ = node_->get_logger();
  base_frame_ = costmap_ros_->getBaseFrameID();

  getParams();
  loadCritics();

  bool regenerate_noises = false;
  declareParam(name_ + ".regenerate_noises", false);
  node_->get_parameter(name_ + ".regenerate_noises", regenerate_noises);
  noise_generator_.initialize(settings_, regenerate_noises);

  reset();
}

void Optimizer::loadCritics()
{
  // Direct instantiation — add or remove critics here to change scoring behavior.
  // Order matters: critics run in order and can set fail_flag to short-circuit the rest.
  auto add = [&](auto * raw, const std::string & short_name) {
      critics_.emplace_back(raw);
      const std::string full_name = name_ + "." + short_name;
      critics_.back()->on_configure(node_, name_, full_name, costmap_ros_);
      RCLCPP_INFO(logger_, "Loaded critic: %s", short_name.c_str());
    };

  add(new critics::ConstraintCritic(), "ConstraintCritic");
  add(new critics::ObstaclesCritic(), "ObstaclesCritic");
  add(new critics::PathFollowCritic(), "PathFollowCritic");
  add(new critics::PathAlignCritic(), "PathAlignCritic");
  add(new critics::GoalCritic(), "GoalCritic");
}

void Optimizer::shutdown()
{
  noise_generator_.shutdown();
}

void Optimizer::getParams()
{
  auto & s = settings_;
  auto getParam = getParamGetter(name_);

  getParam(s.model_dt, "model_dt", 0.05f);
  getParam(s.time_steps, "time_steps", 56);
  getParam(s.batch_size, "batch_size", 1000);
  getParam(s.iteration_count, "iteration_count", 1);
  getParam(s.temperature, "temperature", 0.3f);
  getParam(s.gamma, "gamma", 0.015f);
  getParam(s.base_constraints.vx_max, "vx_max", 0.5f);
  getParam(s.base_constraints.vx_min, "vx_min", -0.35f);
  getParam(s.base_constraints.wz, "wz_max", 1.9f);
  getParam(s.base_constraints.ax_max, "ax_max", 3.0f);
  getParam(s.base_constraints.ax_min, "ax_min", -3.0f);
  getParam(s.base_constraints.az_max, "az_max", 3.5f);
  getParam(s.sampling_std.vx, "vx_std", 0.2f);
  getParam(s.sampling_std.wz, "wz_std", 0.4f);
  getParam(s.retry_attempt_limit, "retry_attempt_limit", 1);
  getParam(s.open_loop, "open_loop", false);
  getParam(s.visualize, "visualize", false);
  getParam(goal_dist_tolerance_, "goal_dist_tolerance", 0.25f);

  s.base_constraints.ax_max = fabs(s.base_constraints.ax_max);
  if (s.base_constraints.ax_min > 0.0f) {
    s.base_constraints.ax_min = -s.base_constraints.ax_min;
    RCLCPP_WARN(logger_, "ax_min sign incorrect, setting negative.");
  }

  float min_turning_r = 0.2f;
  getParam(min_turning_r, "AckermannConstraints.min_turning_r", 0.2f);
  motion_model_ = std::make_shared<AckermannMotionModel>(min_turning_r);

  s.constraints = s.base_constraints;

  // Determine if control period matches model_dt to enable sequence shifting
  double controller_frequency = 10.0;
  declareParam("controller_frequency", 10.0);
  node_->get_parameter("controller_frequency", controller_frequency);
  const double controller_period = 1.0 / controller_frequency;
  constexpr double eps = 1e-6;
  if (std::abs(controller_period - s.model_dt) < eps) {
    s.shift_control_sequence = true;
    RCLCPP_INFO(logger_, "Control sequence shifting enabled (controller_period == model_dt).");
  } else if (controller_period > s.model_dt + eps) {
    RCLCPP_WARN(
      logger_,
      "controller_frequency (%.2f Hz, period=%.4f s) > model_dt (%.4f s). "
      "Set controller_frequency = 1/model_dt for best performance.",
      controller_frequency, controller_period, s.model_dt);
  }
}

void Optimizer::reset(bool reset_dynamic_speed_limits)
{
  state_.reset(settings_.batch_size, settings_.time_steps);
  control_sequence_.reset(settings_.time_steps);
  control_history_.fill({});

  if (settings_.open_loop) {
    last_command_vel_ = geometry_msgs::msg::Twist();
  }

  if (reset_dynamic_speed_limits) {
    settings_.constraints = settings_.base_constraints;
  }

  costs_.setZero(settings_.batch_size);
  generated_trajectories_.reset(settings_.batch_size, settings_.time_steps);
  noise_generator_.reset(settings_);
  motion_model_->initialize(settings_.constraints, settings_.model_dt);

  // Update critic_data_ references (they hold refs to member variables)
  critics_data_.goal_dist_tolerance = goal_dist_tolerance_;

  RCLCPP_INFO(logger_, "Optimizer reset");
}

bool Optimizer::isSpeedLimitActive() const
{
  const auto & base = settings_.base_constraints;
  const auto & curr = settings_.constraints;
  return base.vx_max != curr.vx_max ||
         base.vx_min != curr.vx_min ||
         base.wz != curr.wz;
}

std::tuple<geometry_msgs::msg::TwistStamped, Eigen::ArrayXXf> Optimizer::evalControl(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  const nav_msgs::msg::Path & plan,
  const geometry_msgs::msg::Pose & goal)
{
  prepare(robot_pose, robot_speed, plan, goal);
  Eigen::ArrayXXf optimal_trajectory;

  do {
    optimize();
    optimal_trajectory = getOptimizedTrajectory();
  } while (fallback(critics_data_.fail_flag));

  auto control = getControlFromSequenceAsTwist(plan.header.stamp);
  last_command_vel_ = control.twist;

  if (settings_.shift_control_sequence) {
    shiftControlSequence();
  }

  return std::make_tuple(control, optimal_trajectory);
}

void Optimizer::optimize()
{
  for (size_t i = 0; i < settings_.iteration_count; ++i) {
    generateNoisedTrajectories();
    evalTrajectoriesScores();
    updateControlSequence();
  }
}

void Optimizer::evalTrajectoriesScores()
{
  critic_costs_.clear();

  for (auto & critic : critics_) {
    if (critics_data_.fail_flag) {break;}

    if (settings_.visualize) {
      Eigen::ArrayXf costs_before = critics_data_.costs;
      critic->score(critics_data_);
      critic_costs_.emplace_back(critic->getName(), critics_data_.costs - costs_before);
    } else {
      critic->score(critics_data_);
    }
  }
}

bool Optimizer::fallback(bool fail)
{
  if (!fail) {
    fallback_counter_ = 0;
    return false;
  }

  reset(false);

  if (++fallback_counter_ > settings_.retry_attempt_limit) {
    fallback_counter_ = 0;
    throw std::runtime_error("AckermannMPPI: all trajectories in collision, no valid control.");
  }
  return true;
}

void Optimizer::prepare(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  const nav_msgs::msg::Path & plan,
  const geometry_msgs::msg::Pose & goal)
{
  state_.pose = robot_pose;
  state_.speed = settings_.open_loop ? last_command_vel_ : robot_speed;

  // Compute approximate path length (sum of Euclidean segment distances)
  float path_length = 0.0f;
  for (size_t i = 1; i < plan.poses.size(); ++i) {
    float dx = plan.poses[i].pose.position.x - plan.poses[i - 1].pose.position.x;
    float dy = plan.poses[i].pose.position.y - plan.poses[i - 1].pose.position.y;
    path_length += sqrtf(dx * dx + dy * dy);
  }
  state_.local_path_length = path_length;

  path_ = utils::toTensor(plan);
  costs_.setZero(settings_.batch_size);
  goal_ = goal;

  critics_data_.fail_flag = false;
  critics_data_.motion_model = motion_model_;
  critics_data_.furthest_reached_path_point.reset();
  critics_data_.path_pts_valid.reset();
  critics_data_.trajectories_in_collision.clear();
}

void Optimizer::shiftControlSequence()
{
  auto size = control_sequence_.vx.size();
  utils::shiftColumnsByOnePlace(control_sequence_.vx, -1);
  utils::shiftColumnsByOnePlace(control_sequence_.wz, -1);
  control_sequence_.vx(size - 1) = control_sequence_.vx(size - 2);
  control_sequence_.wz(size - 1) = control_sequence_.wz(size - 2);
}

void Optimizer::generateNoisedTrajectories()
{
  noise_generator_.setNoisedControls(state_, control_sequence_);
  noise_generator_.generateNextNoises();
  updateStateVelocities(state_);
  integrateStateVelocities(generated_trajectories_, state_);
}

void Optimizer::applyControlSequenceConstraints()
{
  auto & s = settings_;

  float max_delta_vx = s.model_dt * s.constraints.ax_max;
  float min_delta_vx = s.model_dt * s.constraints.ax_min;
  float max_delta_wz = s.model_dt * s.constraints.az_max;

  float vx_last = utils::clamp(s.constraints.vx_min, s.constraints.vx_max,
      control_sequence_.vx(0));
  float wz_last = utils::clamp(-s.constraints.wz, s.constraints.wz, control_sequence_.wz(0));
  control_sequence_.vx(0) = vx_last;
  control_sequence_.wz(0) = wz_last;

  for (unsigned int i = 1; i != control_sequence_.vx.size(); i++) {
    float & vx_curr = control_sequence_.vx(i);
    vx_curr = utils::clamp(s.constraints.vx_min, s.constraints.vx_max, vx_curr);
    if (vx_last > 0) {
      vx_curr = utils::clamp(vx_last + min_delta_vx, vx_last + max_delta_vx, vx_curr);
    } else {
      vx_curr = utils::clamp(vx_last - max_delta_vx, vx_last - min_delta_vx, vx_curr);
    }
    vx_last = vx_curr;

    float & wz_curr = control_sequence_.wz(i);
    wz_curr = utils::clamp(-s.constraints.wz, s.constraints.wz, wz_curr);
    wz_curr = utils::clamp(wz_last - max_delta_wz, wz_last + max_delta_wz, wz_curr);
    wz_last = wz_curr;
  }

  motion_model_->applyConstraints(control_sequence_);
}

void Optimizer::updateStateVelocities(models::State & state) const
{
  updateInitialStateVelocities(state);
  propagateStateVelocitiesFromInitials(state);
}

void Optimizer::updateInitialStateVelocities(models::State & state) const
{
  state.vx.col(0) = static_cast<float>(state.speed.linear.x);
  state.wz.col(0) = static_cast<float>(state.speed.angular.z);
  // vy is always zero for Ackermann (non-holonomic)
}

void Optimizer::propagateStateVelocitiesFromInitials(models::State & state) const
{
  motion_model_->predict(state);
}

void Optimizer::integrateStateVelocities(
  Eigen::Array<float, Eigen::Dynamic, 3> & trajectory,
  const Eigen::ArrayXXf & sequence) const
{
  float initial_yaw = static_cast<float>(tf2::getYaw(state_.pose.pose.orientation));

  const auto vx = sequence.col(0);
  const auto wz = sequence.col(1);
  auto traj_x = trajectory.col(0);
  auto traj_y = trajectory.col(1);
  auto traj_yaws = trajectory.col(2);

  const size_t n_size = traj_yaws.size();
  if (n_size == 0) {return;}

  float last_yaw = initial_yaw;
  for (size_t i = 0; i != n_size; i++) {
    last_yaw += wz(i) * settings_.model_dt;
    traj_yaws(i) = last_yaw;
  }

  Eigen::ArrayXf yaw_cos = traj_yaws.cos();
  Eigen::ArrayXf yaw_sin = traj_yaws.sin();
  utils::shiftColumnsByOnePlace(yaw_cos, 1);
  utils::shiftColumnsByOnePlace(yaw_sin, 1);
  yaw_cos(0) = cosf(initial_yaw);
  yaw_sin(0) = sinf(initial_yaw);

  float last_x = state_.pose.pose.position.x;
  float last_y = state_.pose.pose.position.y;
  for (size_t i = 0; i != n_size; i++) {
    last_x += vx(i) * yaw_cos(i) * settings_.model_dt;
    last_y += vx(i) * yaw_sin(i) * settings_.model_dt;
    traj_x(i) = last_x;
    traj_y(i) = last_y;
  }
}

void Optimizer::integrateStateVelocities(
  models::Trajectories & trajectories,
  const models::State & state) const
{
  auto initial_yaw = static_cast<float>(tf2::getYaw(state.pose.pose.orientation));
  const size_t n_cols = trajectories.yaws.cols();

  Eigen::ArrayXf last_yaws = Eigen::ArrayXf::Constant(trajectories.yaws.rows(), initial_yaw);
  for (size_t i = 0; i != n_cols; i++) {
    last_yaws += state.wz.col(i) * settings_.model_dt;
    trajectories.yaws.col(i) = last_yaws;
  }

  Eigen::ArrayXXf yaw_cos = trajectories.yaws.cos();
  Eigen::ArrayXXf yaw_sin = trajectories.yaws.sin();
  utils::shiftColumnsByOnePlace(yaw_cos, 1);
  utils::shiftColumnsByOnePlace(yaw_sin, 1);
  yaw_cos.col(0) = cosf(initial_yaw);
  yaw_sin.col(0) = sinf(initial_yaw);

  // Ackermann: no vy term
  auto dx = (state.vx * yaw_cos).eval();
  auto dy = (state.vx * yaw_sin).eval();

  Eigen::ArrayXf last_x = Eigen::ArrayXf::Constant(
    trajectories.x.rows(), state.pose.pose.position.x);
  Eigen::ArrayXf last_y = Eigen::ArrayXf::Constant(
    trajectories.y.rows(), state.pose.pose.position.y);

  for (size_t i = 0; i != n_cols; i++) {
    last_x += dx.col(i) * settings_.model_dt;
    last_y += dy.col(i) * settings_.model_dt;
    trajectories.x.col(i) = last_x;
    trajectories.y.col(i) = last_y;
  }
}

Eigen::ArrayXXf Optimizer::getOptimizedTrajectory()
{
  // Build [time_steps x 2] sequence (vx, wz) and integrate to poses
  Eigen::ArrayXXf sequence(settings_.time_steps, 2);
  Eigen::Array<float, Eigen::Dynamic, 3> trajectory(settings_.time_steps, 3);

  sequence.col(0) = control_sequence_.vx;
  sequence.col(1) = control_sequence_.wz;

  integrateStateVelocities(trajectory, sequence);
  return trajectory;
}

void Optimizer::updateControlSequence()
{
  auto & s = settings_;

  auto vx_T = control_sequence_.vx.transpose();
  auto bounded_noises_vx = state_.cvx.rowwise() - vx_T;
  const float gamma_vx = s.gamma / (s.sampling_std.vx * s.sampling_std.vx);
  costs_ += (gamma_vx * (bounded_noises_vx.rowwise() * vx_T).rowwise().sum()).eval();

  if (s.sampling_std.wz > 0.0f) {
    auto wz_T = control_sequence_.wz.transpose();
    auto bounded_noises_wz = state_.cwz.rowwise() - wz_T;
    const float gamma_wz = s.gamma / (s.sampling_std.wz * s.sampling_std.wz);
    costs_ += (gamma_wz * (bounded_noises_wz.rowwise() * wz_T).rowwise().sum()).eval();
  }

  auto costs_normalized = costs_ - costs_.minCoeff();
  const float inv_temp = 1.0f / s.temperature;
  auto softmaxes = (-inv_temp * costs_normalized).exp().eval();
  softmaxes /= softmaxes.sum();

  auto softmax_mat = softmaxes.matrix();
  control_sequence_.vx = state_.cvx.transpose().matrix() * softmax_mat;
  control_sequence_.wz = state_.cwz.transpose().matrix() * softmax_mat;

  utils::savitskyGolayFilter(control_sequence_, control_history_, settings_);
  applyControlSequenceConstraints();
}

geometry_msgs::msg::TwistStamped Optimizer::getControlFromSequenceAsTwist(
  const builtin_interfaces::msg::Time & stamp)
{
  unsigned int offset = settings_.shift_control_sequence ? 1 : 0;
  return utils::toTwistStamped(
    control_sequence_.vx(offset),
    control_sequence_.wz(offset),
    stamp, base_frame_);
}

void Optimizer::setSpeedLimit(double speed_limit, bool percentage)
{
  auto & s = settings_;
  // nav2 costmap speed filter sentinel: 255.0 means "no limit active".
  // Matches nav2_costmap_2d::NO_SPEED_LIMIT from filter_values.hpp.
  constexpr double NO_SPEED_LIMIT = 255.0;
  if (speed_limit == NO_SPEED_LIMIT) {
    s.constraints.vx_max = s.base_constraints.vx_max;
    s.constraints.vx_min = s.base_constraints.vx_min;
    s.constraints.wz = s.base_constraints.wz;
  } else {
    double ratio = percentage ? speed_limit / 100.0 : speed_limit / s.base_constraints.vx_max;
    s.constraints.vx_max = s.base_constraints.vx_max * ratio;
    s.constraints.vx_min = s.base_constraints.vx_min * ratio;
    s.constraints.wz = s.base_constraints.wz * ratio;
  }
  motion_model_->initialize(settings_.constraints, settings_.model_dt);
}

}  // namespace mppi
