// Copyright (c) 2025 UMD Loop
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

#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "athena_smac_planner/smac_planner_hybrid.hpp"
#include "msgs/msg/planner_event.hpp"

class GlobalPlannerNode : public rclcpp::Node
{
public:
  explicit GlobalPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("global_planner", options)
  {
  }

  void configure()
  {
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "global_costmap",
      get_namespace(),
      "global_costmap");
    costmap_ros_->set_parameter(rclcpp::Parameter("use_sim_time", get_parameter("use_sim_time").as_bool()));
    // Empty plugin list: rcl_yaml_param_parser cannot represent an empty YAML
    // sequence (plugins: []) — it produces a null rcl_variant_s that crashes
    // NodeParameters. Set it programmatically instead.
    costmap_ros_->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{}));
    costmap_ros_->configure();
    costmap_ros_->activate();

    planner_ = std::make_shared<athena_smac_planner::SmacPlannerHybrid>(
      shared_from_this(), costmap_ros_);

    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/global_path", rclcpp::QoS(1).transient_local());
    event_pub_ = create_publisher<msgs::msg::PlannerEvent>("/planner_event", 10);

    robot_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/robot_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::optional<geometry_msgs::msg::PoseStamped> pending;
        {
          std::lock_guard<std::mutex> lk(mutex_);
          robot_pose_ = *msg;
          // If a goal arrived before robot_pose was ready, plan now
          if (pending_goal_.has_value()) {
            pending = *pending_goal_;
            pending_goal_.reset();
          }
        }
        if (pending.has_value()) {
          RCLCPP_INFO(get_logger(), "[global_planner] robot_pose received — planning pending goal");
          plan(*pending);
        }
      });

    goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { onGoal(msg); });

    RCLCPP_INFO(get_logger(), "GlobalPlannerNode configured.");
  }

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> getCostmapROS()
  {
    return costmap_ros_;
  }

private:
  void publishEvent(uint8_t event_type)
  {
    msgs::msg::PlannerEvent ev;
    ev.event = event_type;
    event_pub_->publish(ev);
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr goal)
  {
    publishEvent(msgs::msg::PlannerEvent::NEW_GOAL);

    {
      std::lock_guard<std::mutex> lk(mutex_);
      if (!robot_pose_.has_value()) {
        RCLCPP_WARN(get_logger(),
          "[global_planner] goal received but no robot_pose yet — waiting");
        pending_goal_ = *goal;   // cache; plan() called once robot_pose arrives
        return;
      }
      // Overwrite any stale pending goal — the new goal supersedes it
      pending_goal_.reset();
    }

    plan(*goal);
  }

  void plan(const geometry_msgs::msg::PoseStamped & goal)
  {
    geometry_msgs::msg::PoseStamped start;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      start = robot_pose_.value();
    }

    publishEvent(msgs::msg::PlannerEvent::PLANNING);

    try {
      auto path = planner_->createPlan(start, goal, [] { return false; });
      path_pub_->publish(path);
      publishEvent(msgs::msg::PlannerEvent::PLAN_SUCCEEDED);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Planning failed: %s", ex.what());
      publishEvent(msgs::msg::PlannerEvent::PLAN_FAILED);
    }
  }

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<athena_smac_planner::SmacPlannerHybrid> planner_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<msgs::msg::PlannerEvent>::SharedPtr event_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;

  std::mutex mutex_;
  std::optional<geometry_msgs::msg::PoseStamped> robot_pose_;
  std::optional<geometry_msgs::msg::PoseStamped> pending_goal_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<GlobalPlannerNode>();
  node->configure();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(node->getCostmapROS()->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
