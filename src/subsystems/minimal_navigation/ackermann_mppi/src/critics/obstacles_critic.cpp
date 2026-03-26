// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Open Navigation LLC
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

#include <cmath>
#include "ackermann_mppi/critics/obstacles_critic.hpp"

namespace mppi::critics
{

void ObstaclesCritic::initialize()
{
  auto getParam = getParamGetter(name_);
  getParam(consider_footprint_, "consider_footprint", false);
  getParam(power_, "cost_power", 1);
  getParam(repulsion_weight_, "repulsion_weight", 1.5f);
  getParam(critical_weight_, "critical_weight", 20.0f);
  getParam(collision_cost_, "collision_cost", 100000.0f);
  getParam(collision_margin_distance_, "collision_margin_distance", 0.10f);
  getParam(near_goal_distance_, "near_goal_distance", 0.5f);
  getParam(inflation_layer_name_, "inflation_layer_name", std::string(""));

  collision_checker_.setCostmap(costmap_);
  possible_collision_cost_ = findCircumscribedCost(costmap_ros_);

  if (possible_collision_cost_ < 1.0f) {
    RCLCPP_ERROR(
      logger_,
      "Inflation layer either not found or inflation radius is not set sufficiently for "
      "non-circular collision checking. Set inflation_radius >= half of the robot's largest "
      "cross-section. This will substantially impact run-time performance.");
  }

  RCLCPP_INFO(
    logger_,
    "ObstaclesCritic instantiated with %d power and %f / %f weights. "
    "Collision check based on %s cost.",
    power_, critical_weight_, repulsion_weight_,
    consider_footprint_ ? "footprint" : "circular");
}

float ObstaclesCritic::findCircumscribedCost(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap)
{
  double result = -1.0;
  const double circum_radius = costmap->getLayeredCostmap()->getCircumscribedRadius();
  if (static_cast<float>(circum_radius) == circumscribed_radius_) {
    return circumscribed_cost_;
  }

  std::shared_ptr<nav2_costmap_2d::InflationLayer> inflation_layer;
  for (auto & layer : *costmap->getLayeredCostmap()->getPlugins()) {
    auto candidate = std::dynamic_pointer_cast<nav2_costmap_2d::InflationLayer>(layer);
    if (candidate &&
      (inflation_layer_name_.empty() || candidate->getName() == inflation_layer_name_))
    {
      inflation_layer = candidate;
      break;
    }
  }
  if (inflation_layer != nullptr) {
    const double resolution = costmap->getCostmap()->getResolution();
    result = inflation_layer->computeCost(circum_radius / resolution);
    const std::string & ln = inflation_layer->getName();
    rclcpp::Parameter p;
    if (costmap->get_parameter(ln + ".cost_scaling_factor", p)) {
      inflation_scale_factor_ = static_cast<float>(p.as_double());
    }
    if (costmap->get_parameter(ln + ".inflation_radius", p)) {
      inflation_radius_ = static_cast<float>(p.as_double());
    }
  } else {
    RCLCPP_WARN(
      logger_,
      "No inflation layer found. Cannot use costmap potential field for fast collision "
      "pre-screening. Only absolute collisions will be detected.");
  }

  circumscribed_radius_ = static_cast<float>(circum_radius);
  circumscribed_cost_ = static_cast<float>(result);
  return circumscribed_cost_;
}

float ObstaclesCritic::distanceToObstacle(const CollisionCost & cost)
{
  const float scale_factor = inflation_scale_factor_;
  const float min_radius = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
  // nav2 inflation layer maps INSCRIBED_INFLATED_OBSTACLE (254) - 1 = 253 to the inscribed
  // radius. We invert the exponential cost formula to recover metric distance.
  constexpr float kNavInscribedCostThreshold = 253.0f;
  float dist_to_obj =
    (scale_factor * min_radius - logf(cost.cost) + logf(kNavInscribedCostThreshold)) /
    scale_factor;
  if (!cost.using_footprint) {
    dist_to_obj -= min_radius;
  }
  return dist_to_obj;
}

void ObstaclesCritic::score(CriticData & data)
{
  if (!enabled_) {return;}

  if (consider_footprint_) {
    possible_collision_cost_ = findCircumscribedCost(costmap_ros_);
  }

  bool near_goal = data.state.local_path_length < near_goal_distance_;

  Eigen::ArrayXf raw_cost = Eigen::ArrayXf::Zero(data.costs.size());
  Eigen::ArrayXf repulsive_cost = Eigen::ArrayXf::Zero(data.costs.size());

  const unsigned int traj_len = data.trajectories.x.cols();
  const unsigned int batch_size = data.trajectories.x.rows();
  bool all_trajectories_collide = true;

  auto & collisions = data.trajectories_in_collision;
  const bool track_collisions = !collisions.empty();

  for (unsigned int i = 0; i != batch_size; i++) {
    bool trajectory_collide = false;
    float traj_cost = 0.0f;
    const auto & traj = data.trajectories;
    CollisionCost pose_cost;

    for (unsigned int j = 0; j != traj_len; j++) {
      pose_cost = costAtPose(traj.x(i, j), traj.y(i, j), traj.yaws(i, j));
      if (pose_cost.cost < 1.0f) {continue;}

      if (inCollision(pose_cost.cost)) {
        trajectory_collide = true;
        break;
      }

      if (inflation_radius_ == 0.0f || inflation_scale_factor_ == 0.0f) {continue;}

      const float dist_to_obj = distanceToObstacle(pose_cost);
      if (dist_to_obj < collision_margin_distance_) {
        traj_cost += (collision_margin_distance_ - dist_to_obj);
      }
      if (!near_goal) {
        repulsive_cost[i] += inflation_radius_ - dist_to_obj;
      }
    }

    if (!trajectory_collide) {all_trajectories_collide = false;}
    raw_cost(i) = trajectory_collide ? collision_cost_ : traj_cost;
    if (trajectory_collide && track_collisions) {collisions[i] = true;}
  }

  auto repulsive_cost_normalized = (repulsive_cost - repulsive_cost.minCoeff()) / traj_len;

  if (power_ > 1u) {
    data.costs +=
      ((critical_weight_ * raw_cost) + (repulsion_weight_ * repulsive_cost_normalized))
      .pow(power_);
  } else {
    data.costs +=
      (critical_weight_ * raw_cost) + (repulsion_weight_ * repulsive_cost_normalized);
  }

  data.fail_flag = all_trajectories_collide;
}

bool ObstaclesCritic::inCollision(float cost) const
{
  bool is_tracking_unknown = costmap_ros_->getLayeredCostmap()->isTrackingUnknown();
  using namespace nav2_costmap_2d;  // NOLINT
  switch (static_cast<unsigned char>(cost)) {
    case (LETHAL_OBSTACLE):
      return true;
    case (INSCRIBED_INFLATED_OBSTACLE):
      return consider_footprint_ ? false : true;
    case (NO_INFORMATION):
      return is_tracking_unknown ? false : true;
  }
  return false;
}

CollisionCost ObstaclesCritic::costAtPose(float x, float y, float theta)
{
  CollisionCost collision_cost;
  float & cost = collision_cost.cost;
  collision_cost.using_footprint = false;

  unsigned int x_i, y_i;
  if (!collision_checker_.worldToMap(x, y, x_i, y_i)) {
    cost = nav2_costmap_2d::NO_INFORMATION;
    return collision_cost;
  }
  cost = collision_checker_.pointCost(x_i, y_i);

  if (consider_footprint_ &&
    (cost >= possible_collision_cost_ || possible_collision_cost_ < 1.0f))
  {
    cost = static_cast<float>(collision_checker_.footprintCostAtPose(
        x, y, theta, costmap_ros_->getRobotFootprint()));
    collision_cost.using_footprint = true;
  }
  return collision_cost;
}

}  // namespace mppi::critics
