// Copyright (c) 2020, Samsung Research America
// Copyright (c) 2023, Open Navigation LLC
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

#include <string>
#include <memory>
#include <vector>
#include <algorithm>
#include <limits>
#include <stdexcept>

#include "athena_smac_planner/smac_planner_hybrid.hpp"

// #define BENCHMARK_TESTING

namespace athena_smac_planner
{

using namespace std::chrono;  // NOLINT

SmacPlannerHybrid::SmacPlannerHybrid(
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
  const std::string & name)
: _a_star(nullptr),
  _collision_checker(nullptr, 1, nullptr),
  _smoother(nullptr),
  _costmap(nullptr),
  _costmap_downsampler(nullptr)
{
  _name = name;
  _logger = node->get_logger();
  _clock = node->get_clock();
  _costmap = costmap_ros->getCostmap();
  _costmap_ros = costmap_ros;
  _global_frame = costmap_ros->getGlobalFrameID();

  RCLCPP_INFO(_logger, "Configuring %s of type SmacPlannerHybrid", _name.c_str());

  // Helper: declare param if absent then get its value
  auto p = [&node, &name](const std::string & param, auto def) {
      if (!node->has_parameter(name + "." + param)) {
        node->declare_parameter(name + "." + param, def);
      }
      return node->get_parameter(name + "." + param).get_value<decltype(def)>();
    };

  int angle_quantizations;
  double analytic_expansion_max_length_m;
  bool smooth_path;

  // General planner params
  _downsample_costmap = p("downsample_costmap", false);
  _downsampling_factor = p("downsampling_factor", 1);

  angle_quantizations = p("angle_quantization_bins", 72);
  _angle_bin_size = 2.0 * M_PI / angle_quantizations;
  _angle_quantizations = static_cast<unsigned int>(angle_quantizations);

  _tolerance = static_cast<float>(p("tolerance", 0.25));
  _allow_unknown = p("allow_unknown", true);
  _max_iterations = p("max_iterations", 1000000);
  _max_on_approach_iterations = p("max_on_approach_iterations", 1000);
  _terminal_checking_interval = p("terminal_checking_interval", 5000);
  smooth_path = p("smooth_path", true);

  _minimum_turning_radius_global_coords = p("minimum_turning_radius", 0.4);
  _search_info.allow_primitive_interpolation = p("allow_primitive_interpolation", false);
  _search_info.cache_obstacle_heuristic = p("cache_obstacle_heuristic", false);
  _search_info.reverse_penalty = p("reverse_penalty", 2.0);
  _search_info.change_penalty = p("change_penalty", 0.0);
  _search_info.non_straight_penalty = p("non_straight_penalty", 1.2);
  _search_info.cost_penalty = p("cost_penalty", 2.0);
  _search_info.retrospective_penalty = p("retrospective_penalty", 0.015);
  _search_info.analytic_expansion_ratio = p("analytic_expansion_ratio", 3.5);
  _search_info.analytic_expansion_max_cost = p("analytic_expansion_max_cost", 200.0);
  _search_info.analytic_expansion_max_cost_override =
    p("analytic_expansion_max_cost_override", false);
  _search_info.use_quadratic_cost_penalty = p("use_quadratic_cost_penalty", false);
  _search_info.downsample_obstacle_heuristic = p("downsample_obstacle_heuristic", true);

  analytic_expansion_max_length_m = p("analytic_expansion_max_length", 3.0);
  _search_info.analytic_expansion_max_length =
    analytic_expansion_max_length_m / _costmap->getResolution();

  _max_planning_time = p("max_planning_time", 5.0);
  _lookup_table_size = p("lookup_table_size", 20.0);
  _debug_visualizations = p("debug_visualizations", false);
  _motion_model_for_search = p("motion_model_for_search", std::string("DUBIN"));

  std::string goal_heading_type = p("goal_heading_mode", std::string("DEFAULT"));
  _goal_heading_mode = fromStringToGH(goal_heading_type);

  _coarse_search_resolution = p("coarse_search_resolution", 1);

  if (_goal_heading_mode == GoalHeadingMode::UNKNOWN) {
    throw std::runtime_error(
            "Unable to get GoalHeader type. Given '" + goal_heading_type + "' "
            "Valid options are DEFAULT, BIDIRECTIONAL, ALL_DIRECTION.");
  }

  _motion_model = fromString(_motion_model_for_search);

  if (_motion_model == MotionModel::UNKNOWN) {
    RCLCPP_WARN(
      _logger,
      "Unable to get MotionModel search type. Given '%s', "
      "valid options are MOORE, VON_NEUMANN, DUBIN, REEDS_SHEPP, STATE_LATTICE.",
      _motion_model_for_search.c_str());
  }

  if (_max_on_approach_iterations <= 0) {
    RCLCPP_WARN(
      _logger, "On approach iteration selected as <= 0, "
      "disabling tolerance and on approach iterations.");
    _max_on_approach_iterations = std::numeric_limits<int>::max();
  }

  if (_max_iterations <= 0) {
    RCLCPP_WARN(
      _logger, "maximum iteration selected as <= 0, "
      "disabling maximum iterations.");
    _max_iterations = std::numeric_limits<int>::max();
  }

  if (_coarse_search_resolution <= 0) {
    RCLCPP_WARN(
      _logger, "coarse iteration resolution selected as <= 0, "
      "disabling coarse iteration resolution search for goal heading");
    _coarse_search_resolution = 1;
  }

  if (_angle_quantizations % _coarse_search_resolution != 0) {
    throw std::runtime_error(
            "coarse iteration should be an increment of the number of angular bins configured");
  }

  if (_minimum_turning_radius_global_coords < _costmap->getResolution() * _downsampling_factor) {
    RCLCPP_WARN(
      _logger, "Min turning radius cannot be less than the search grid cell resolution!");
    _minimum_turning_radius_global_coords = _costmap->getResolution() * _downsampling_factor;
  }

  // Convert to grid coordinates
  if (!_downsample_costmap) {
    _downsampling_factor = 1;
  }
  _search_info.minimum_turning_radius =
    _minimum_turning_radius_global_coords / (_costmap->getResolution() * _downsampling_factor);
  _lookup_table_dim =
    static_cast<float>(_lookup_table_size) /
    static_cast<float>(_costmap->getResolution() * _downsampling_factor);

  // Make sure its a whole number
  _lookup_table_dim = static_cast<float>(static_cast<int>(_lookup_table_dim));

  // Make sure its an odd number
  if (static_cast<int>(_lookup_table_dim) % 2 == 0) {
    RCLCPP_INFO(
      _logger,
      "Even sized heuristic lookup table size set %f, increasing size by 1 to make odd",
      _lookup_table_dim);
    _lookup_table_dim += 1.0;
  }

  // Initialize collision checker
  _collision_checker = GridCollisionChecker(
    _costmap_ros, _angle_quantizations, node);
  _collision_checker.setFootprint(
    costmap_ros->getRobotFootprint(),
    costmap_ros->getUseRadius(),
    findCircumscribedCost(_costmap_ros));

  // Initialize A* template
  _a_star = std::make_unique<AStarAlgorithm<NodeHybrid>>(_motion_model, _search_info);
  _a_star->initialize(
    _allow_unknown,
    _max_iterations,
    _max_on_approach_iterations,
    _terminal_checking_interval,
    _max_planning_time,
    _lookup_table_dim,
    _angle_quantizations);

  // Initialize path smoother
  if (smooth_path) {
    SmootherParams smoother_params;
    smoother_params.get(node.get(), _name);
    _smoother = std::make_unique<Smoother>(smoother_params);
    _smoother->initialize(_minimum_turning_radius_global_coords);
  }

  // Initialize costmap downsampler
  if (_downsample_costmap && _downsampling_factor > 1) {
    _costmap_downsampler = std::make_unique<CostmapDownsampler>();
    _costmap_downsampler->on_configure(_costmap, _downsampling_factor);
  }

  // Create publishers
  _raw_plan_publisher = node->create_publisher<nav_msgs::msg::Path>("unsmoothed_plan", 1);

  if (_debug_visualizations) {
    _expansions_publisher =
      node->create_publisher<geometry_msgs::msg::PoseArray>("expansions", 1);
    _planned_footprints_publisher =
      node->create_publisher<visualization_msgs::msg::MarkerArray>("planned_footprints", 1);
    _smoothed_footprints_publisher =
      node->create_publisher<visualization_msgs::msg::MarkerArray>("smoothed_footprints", 1);
  }

  RCLCPP_INFO(
    _logger, "Configured %s of type SmacPlannerHybrid with "
    "maximum iterations %i, max on approach iterations %i, and %s. Tolerance %.2f."
    "Using motion model: %s.",
    _name.c_str(), _max_iterations, _max_on_approach_iterations,
    _allow_unknown ? "allowing unknown traversal" : "not allowing unknown traversal",
    _tolerance, toString(_motion_model).c_str());
}

SmacPlannerHybrid::~SmacPlannerHybrid()
{
  RCLCPP_INFO(_logger, "Destroying %s of type SmacPlannerHybrid", _name.c_str());
  _a_star.reset();
  _smoother.reset();
  if (_costmap_downsampler) {
    _costmap_downsampler->on_cleanup();
    _costmap_downsampler.reset();
  }
}

nav_msgs::msg::Path SmacPlannerHybrid::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  std::lock_guard<std::mutex> lock_reinit(_mutex);
  steady_clock::time_point a = steady_clock::now();

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(_costmap->getMutex()));

  // Downsample costmap, if required
  nav2_costmap_2d::Costmap2D * costmap = _costmap;
  if (_downsample_costmap && _downsampling_factor > 1) {
    costmap = _costmap_downsampler->downsample(_downsampling_factor);
    _collision_checker.setCostmap(costmap);
  }

  // Set collision checker and costmap information
  _collision_checker.setFootprint(
    _costmap_ros->getRobotFootprint(),
    _costmap_ros->getUseRadius(),
    findCircumscribedCost(_costmap_ros));
  _a_star->setCollisionChecker(&_collision_checker);

  // Set starting point, in A* bin search coordinates
  float mx_start, my_start, mx_goal, my_goal;
  {
    unsigned int umx, umy;
    if (!costmap->worldToMap(start.pose.position.x, start.pose.position.y, umx, umy)) {
      throw std::runtime_error(
              "Start Coordinates of(" + std::to_string(start.pose.position.x) + ", " +
              std::to_string(start.pose.position.y) + ") was outside bounds");
    }
    mx_start = static_cast<float>(umx);
    my_start = static_cast<float>(umy);
  }

  double start_orientation_bin = std::round(tf2::getYaw(start.pose.orientation) / _angle_bin_size);
  while (start_orientation_bin < 0.0) {
    start_orientation_bin += static_cast<float>(_angle_quantizations);
  }
  // This is needed to handle precision issues
  if (start_orientation_bin >= static_cast<float>(_angle_quantizations)) {
    start_orientation_bin -= static_cast<float>(_angle_quantizations);
  }
  unsigned int start_orientation_bin_int =
    static_cast<unsigned int>(start_orientation_bin);
  _a_star->setStart(mx_start, my_start, start_orientation_bin_int);

  // Set goal point, in A* bin search coordinates
  {
    unsigned int umx, umy;
    if (!costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, umx, umy)) {
      throw std::runtime_error(
              "Goal Coordinates of(" + std::to_string(goal.pose.position.x) + ", " +
              std::to_string(goal.pose.position.y) + ") was outside bounds");
    }
    mx_goal = static_cast<float>(umx);
    my_goal = static_cast<float>(umy);
  }
  double goal_orientation_bin = std::round(tf2::getYaw(goal.pose.orientation) / _angle_bin_size);
  while (goal_orientation_bin < 0.0) {
    goal_orientation_bin += static_cast<float>(_angle_quantizations);
  }
  // This is needed to handle precision issues
  if (goal_orientation_bin >= static_cast<float>(_angle_quantizations)) {
    goal_orientation_bin -= static_cast<float>(_angle_quantizations);
  }
  unsigned int goal_orientation_bin_int =
    static_cast<unsigned int>(goal_orientation_bin);
  _a_star->setGoal(
    mx_goal, my_goal, static_cast<unsigned int>(goal_orientation_bin_int),
    _goal_heading_mode, _coarse_search_resolution);

  // Setup message
  nav_msgs::msg::Path plan;
  plan.header.stamp = _clock->now();
  plan.header.frame_id = _global_frame;
  geometry_msgs::msg::PoseStamped pose;
  pose.header = plan.header;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;

  // Corner case of start and goal being on the same cell
  if (std::floor(mx_start) == std::floor(mx_goal) &&
    std::floor(my_start) == std::floor(my_goal) &&
    start_orientation_bin_int == goal_orientation_bin_int)
  {
    pose.pose = start.pose;
    pose.pose.orientation = goal.pose.orientation;
    plan.poses.push_back(pose);

    // Publish raw path for debug
    if (_raw_plan_publisher->get_subscription_count() > 0) {
      auto msg = std::make_unique<nav_msgs::msg::Path>(plan);
      _raw_plan_publisher->publish(std::move(msg));
    }

    return plan;
  }

  // Compute plan
  NodeHybrid::CoordinateVector path;
  int num_iterations = 0;
  std::string error;
  std::unique_ptr<std::vector<std::tuple<float, float, float>>> expansions = nullptr;
  if (_debug_visualizations) {
    expansions = std::make_unique<std::vector<std::tuple<float, float, float>>>();
  }

  if (!_a_star->createPath(
      path, num_iterations,
      _tolerance / static_cast<float>(costmap->getResolution()), cancel_checker, expansions.get()))
  {
    if (_debug_visualizations) {
      auto msg = std::make_unique<geometry_msgs::msg::PoseArray>();
      geometry_msgs::msg::Pose msg_pose;
      msg->header.stamp = _clock->now();
      msg->header.frame_id = _global_frame;
      for (auto & e : *expansions) {
        msg_pose.position.x = std::get<0>(e);
        msg_pose.position.y = std::get<1>(e);
        msg_pose.orientation = getWorldOrientation(std::get<2>(e));
        msg->poses.push_back(msg_pose);
      }
      _expansions_publisher->publish(std::move(msg));
    }

    if (num_iterations == 1) {
      throw std::runtime_error("Start occupied");
    }

    if (num_iterations < _a_star->getMaxIterations()) {
      throw std::runtime_error("No valid path could be found");
    } else {
      throw std::runtime_error("Exceeded maximum iterations");
    }
  }

  // Convert to world coordinates
  plan.poses.reserve(path.size());
  for (int i = path.size() - 1; i >= 0; --i) {
    pose.pose = getWorldCoords(path[i].x, path[i].y, costmap);
    pose.pose.orientation = getWorldOrientation(path[i].theta);
    plan.poses.push_back(pose);
  }

  // Publish raw path for debug
  if (_raw_plan_publisher->get_subscription_count() > 0) {
    auto msg = std::make_unique<nav_msgs::msg::Path>(plan);
    _raw_plan_publisher->publish(std::move(msg));
  }

  if (_debug_visualizations) {
    // Publish expansions for debug
    auto now = _clock->now();
    auto msg = std::make_unique<geometry_msgs::msg::PoseArray>();
    geometry_msgs::msg::Pose msg_pose;
    msg->header.stamp = now;
    msg->header.frame_id = _global_frame;
    for (auto & e : *expansions) {
      msg_pose.position.x = std::get<0>(e);
      msg_pose.position.y = std::get<1>(e);
      msg_pose.orientation = getWorldOrientation(std::get<2>(e));
      msg->poses.push_back(msg_pose);
    }
    _expansions_publisher->publish(std::move(msg));

    if (_planned_footprints_publisher->get_subscription_count() > 0) {
      // Clear all markers first
      auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
      visualization_msgs::msg::Marker clear_all_marker;
      clear_all_marker.action = visualization_msgs::msg::Marker::DELETEALL;
      marker_array->markers.push_back(clear_all_marker);
      _planned_footprints_publisher->publish(std::move(marker_array));

      // Publish planned footprints for debug
      marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
      for (size_t i = 0; i < plan.poses.size(); i++) {
        const std::vector<geometry_msgs::msg::Point> edge =
          transformFootprintToEdges(plan.poses[i].pose, _costmap_ros->getRobotFootprint());
        marker_array->markers.push_back(createMarker(edge, i, _global_frame, now));
      }
      _planned_footprints_publisher->publish(std::move(marker_array));
    }
  }

  // Find how much time we have left to do smoothing
  steady_clock::time_point b = steady_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(b - a);
  double time_remaining = _max_planning_time - static_cast<double>(time_span.count());

#ifdef BENCHMARK_TESTING
  std::cout << "It took " << time_span.count() * 1000 <<
    " milliseconds with " << num_iterations << " iterations." << std::endl;
#endif

  // Smooth plan
  if (_smoother && num_iterations > 1) {
    _smoother->smooth(plan, costmap, time_remaining);
  }

#ifdef BENCHMARK_TESTING
  steady_clock::time_point c = steady_clock::now();
  duration<double> time_span2 = duration_cast<duration<double>>(c - b);
  std::cout << "It took " << time_span2.count() * 1000 <<
    " milliseconds to smooth path." << std::endl;
#endif

  if (_debug_visualizations) {
    if (_smoothed_footprints_publisher->get_subscription_count() > 0) {
      // Clear all markers first
      auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
      visualization_msgs::msg::Marker clear_all_marker;
      clear_all_marker.action = visualization_msgs::msg::Marker::DELETEALL;
      marker_array->markers.push_back(clear_all_marker);
      _smoothed_footprints_publisher->publish(std::move(marker_array));

      // Publish smoothed footprints for debug
      marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
      auto now = _clock->now();
      for (size_t i = 0; i < plan.poses.size(); i++) {
        const std::vector<geometry_msgs::msg::Point> edge =
          transformFootprintToEdges(plan.poses[i].pose, _costmap_ros->getRobotFootprint());
        marker_array->markers.push_back(createMarker(edge, i, _global_frame, now));
      }
      _smoothed_footprints_publisher->publish(std::move(marker_array));
    }
  }

  return plan;
}

}  // namespace athena_smac_planner
