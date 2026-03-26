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

#ifndef ACKERMANN_MPPI__CRITIC_FUNCTION_HPP_
#define ACKERMANN_MPPI__CRITIC_FUNCTION_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "ackermann_mppi/critic_data.hpp"

namespace mppi::critics
{

struct CollisionCost
{
  float cost{0};
  bool using_footprint{false};
};

/**
 * @class mppi::critics::CriticFunction
 * @brief Abstract base for MPPI trajectory scoring critics.
 *
 * Compared to the original nav2 version:
 * - Uses rclcpp::Node::SharedPtr instead of LifecycleNode::WeakPtr
 * - No ParametersHandler dependency — uses a simple getParam() helper inline
 * - Instantiated directly by the Optimizer (no pluginlib)
 */
class CriticFunction
{
public:
  CriticFunction() = default;
  virtual ~CriticFunction() = default;

  /**
   * @brief Configure critic on startup.
   * @param node ROS2 node (used for parameter access and logging)
   * @param parent_name Name of the parent optimizer (for reading shared params)
   * @param name Name of this critic (used as param namespace)
   * @param costmap_ros Costmap for collision/cost queries
   */
  void on_configure(
    rclcpp::Node::SharedPtr node,
    const std::string & parent_name,
    const std::string & name,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    node_ = node;
    logger_ = node_->get_logger();
    name_ = name;
    parent_name_ = parent_name;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();

    // Declare and read the common 'enabled' parameter
    declare_param(name_ + ".enabled", true);
    node_->get_parameter(name_ + ".enabled", enabled_);

    initialize();
  }

  virtual void score(CriticData & data) = 0;
  virtual void initialize() = 0;

  std::string getName() { return name_; }

protected:
  /**
   * @brief Declare a parameter with a default value if not already declared.
   */
  template<typename T>
  void declare_param(const std::string & full_name, T default_value)
  {
    if (!node_->has_parameter(full_name)) {
      node_->declare_parameter(full_name, default_value);
    }
  }

  /**
   * @brief Get a parameter accessor scoped to a given namespace.
   *
   * Usage (same pattern as original ParametersHandler::getParamGetter):
   *   auto getParam = getParamGetter(name_);
   *   getParam(weight_, "cost_weight", 5.0f);
   */
  auto getParamGetter(const std::string & ns)
  {
    return [this, ns](auto & setting, const std::string & name, auto default_value) {
             std::string full_name = ns.empty() ? name : ns + "." + name;
             declare_param(full_name, default_value);
             node_->get_parameter(full_name, setting);
           };
  }

  bool enabled_{true};
  std::string name_, parent_name_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
};

}  // namespace mppi::critics

#endif  // ACKERMANN_MPPI__CRITIC_FUNCTION_HPP_
