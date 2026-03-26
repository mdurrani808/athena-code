// Copyright (c) 2020, Samsung Research America
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
// limitations under the License. Reserved.

#ifndef ATHENA_SMAC_PLANNER__SMAC_PLANNER_HYBRID_HPP_
#define ATHENA_SMAC_PLANNER__SMAC_PLANNER_HYBRID_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "athena_smac_planner/a_star.hpp"
#include "athena_smac_planner/smoother.hpp"
#include "athena_smac_planner/utils.hpp"
#include "athena_smac_planner/costmap_downsampler.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.hpp"

namespace athena_smac_planner
{

class SmacPlannerHybrid
{
public:
  /**
   * @brief Constructor — initializes the planner from ROS parameters.
   * @param node Shared ptr to an rclcpp::Node for parameter reading and publisher creation.
   * @param costmap_ros Costmap2DROS providing the collision costmap.
   * @param name Parameter namespace prefix (default "SmacPlannerHybrid").
   */
  SmacPlannerHybrid(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    const std::string & name = "SmacPlannerHybrid");

  /**
   * @brief Destructor
   */
  ~SmacPlannerHybrid();

  /**
   * @brief Create a path from start to goal.
   * @param start Start pose in map frame.
   * @param goal  Goal pose in map frame.
   * @param cancel_checker Callable that returns true when planning should abort.
   * @return nav_msgs::msg::Path of the generated path.
   */
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker);

protected:
  std::unique_ptr<AStarAlgorithm<NodeHybrid>> _a_star;
  GridCollisionChecker _collision_checker;
  std::unique_ptr<Smoother> _smoother;
  rclcpp::Clock::SharedPtr _clock;
  rclcpp::Logger _logger{rclcpp::get_logger("SmacPlannerHybrid")};
  nav2_costmap_2d::Costmap2D * _costmap;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> _costmap_ros;
  std::unique_ptr<CostmapDownsampler> _costmap_downsampler;
  std::string _global_frame, _name;
  float _lookup_table_dim;
  float _tolerance;
  bool _downsample_costmap;
  int _downsampling_factor;
  double _angle_bin_size;
  unsigned int _angle_quantizations;
  bool _allow_unknown;
  int _max_iterations;
  int _max_on_approach_iterations;
  int _terminal_checking_interval;
  SearchInfo _search_info;
  double _max_planning_time;
  double _lookup_table_size;
  double _minimum_turning_radius_global_coords;
  bool _debug_visualizations;
  std::string _motion_model_for_search;
  MotionModel _motion_model;
  GoalHeadingMode _goal_heading_mode;
  int _coarse_search_resolution;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _raw_plan_publisher;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    _planned_footprints_publisher;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    _smoothed_footprints_publisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr
    _expansions_publisher;
  std::mutex _mutex;
};

}  // namespace athena_smac_planner

#endif  // ATHENA_SMAC_PLANNER__SMAC_PLANNER_HYBRID_HPP_
