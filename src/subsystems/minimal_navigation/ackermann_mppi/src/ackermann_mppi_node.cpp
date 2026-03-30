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

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"

#include "ackermann_mppi/optimizer.hpp"

/**
 * @class AckermannMPPINode
 * @brief Standalone ROS2 node that runs MPPI for an Ackermann steering vehicle.
 *
 * Inputs:
 *   /global_path (nav_msgs/msg/Path)     — global path to follow
 *   /odom        (nav_msgs/msg/Odometry) — robot velocity feedback
 *
 * Outputs:
 *   /cmd_vel     (geometry_msgs/msg/Twist) — velocity command
 *
 * The node also owns a Costmap2DROS instance (subscribes to sensor topics
 * independently). Configure it via the standard nav2 costmap parameters.
 *
 * Key parameters (under the node's namespace):
 *   controller_frequency (double, default 10.0) — control loop rate in Hz
 *   plan_timeout         (double, default 2.0)  — seconds before stale plan triggers stop
 *   mppi.model_dt        (float, default 0.05)  — must equal 1/controller_frequency
 *   mppi.batch_size      (int, default 1000)
 *   mppi.time_steps      (int, default 56)
 *   mppi.vx_max          (float, default 0.5)
 *   mppi.vx_min          (float, default -0.35)
 *   mppi.wz_max          (float, default 1.9)
 *   mppi.AckermannConstraints.min_turning_r (float, default 0.2)
 *   ... (see optimizer.cpp getParams() for full list)
 */
class AckermannMPPINode : public rclcpp::Node
{
public:
  explicit AckermannMPPINode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("ackermann_mppi", options)
  {
    // Declare parameters in constructor so they're available before configure()
    declare_parameter("controller_frequency", 10.0);
    declare_parameter("plan_timeout", 2.0);
    declare_parameter("odom_topic", std::string("/odom"));
    declare_parameter("plan_topic", std::string("/global_path"));
    declare_parameter("nav_enabled_topic", std::string("/nav_enabled"));
    declare_parameter("cmd_vel_topic", std::string("/rear_ackermann_controller/reference"));
    declare_parameter("robot_base_frame", std::string("base_link"));
    declare_parameter("global_frame", std::string("map"));
  }

  /**
   * @brief Finish setup after construction (requires shared_from_this()).
   * Call this immediately after make_shared<AckermannMPPINode>().
   */
  void configure()
  {
    RCLCPP_INFO(get_logger(), "AckermannMPPINode configuring...");

    // Cache frame names — these never change at runtime.
    get_parameter("global_frame", global_frame_);
    get_parameter("robot_base_frame", base_frame_);
    get_parameter("plan_timeout", plan_timeout_s_);

    // TF buffer and listener
    tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

    // Costmap2DROS manages its own ROS2 lifecycle (sensors, inflation layers, etc).
    // It needs to be spun in the same executor as this node.
    // Parameters are read from the "local_costmap" namespace (standard nav2 costmap params).
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "local_costmap",
      get_namespace(),   // parent namespace
      "local_costmap"    // local namespace
    );
    // Transfer the TF buffer so the costmap shares our TF instance
    bool use_sim_time = false;
    get_parameter_or("use_sim_time", use_sim_time, false);
    costmap_ros_->set_parameter(rclcpp::Parameter("use_sim_time", use_sim_time));
    // Empty plugin list: rcl_yaml_param_parser cannot represent an empty YAML
    // sequence (plugins: []) — it produces a null rcl_variant_s that crashes
    // NodeParameters. Set it programmatically instead.
    costmap_ros_->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{}));
    costmap_ros_->configure();
    costmap_ros_->activate();

    optimizer_.initialize(shared_from_this(), "mppi", costmap_ros_, tf_);

    double controller_frequency;
    get_parameter("controller_frequency", controller_frequency);

    // Subscriptions
    std::string odom_topic, plan_topic, nav_enabled_topic;
    get_parameter("odom_topic", odom_topic);
    get_parameter("plan_topic", plan_topic);
    get_parameter("nav_enabled_topic", nav_enabled_topic);

    // NOTE: both callbacks write shared state that is read by the control timer.
    // The mutexes (plan_mutex_, odom_mutex_) below protect against the data race
    // in the MultiThreadedExecutor where these run in parallel with controlLoop().
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        last_odom_ = *msg;
      });

    plan_sub_ = create_subscription<nav_msgs::msg::Path>(
      plan_topic, rclcpp::QoS(1).transient_local(),
      [this](const nav_msgs::msg::Path::SharedPtr msg) {
        if (!msg->poses.empty()) {
          std::lock_guard<std::mutex> lock(plan_mutex_);
          current_plan_ = *msg;
          last_plan_time_ = now();
          RCLCPP_INFO(get_logger(), "[mppi] plan received: %zu poses", msg->poses.size());
        } else {
          RCLCPP_WARN(get_logger(), "[mppi] received empty plan, ignoring");
        }
      });

    nav_enabled_sub_ = create_subscription<std_msgs::msg::Bool>(
      nav_enabled_topic, 1,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        nav_enabled_.store(msg->data);
        RCLCPP_INFO(get_logger(), "[mppi] nav_enabled changed → %s", msg->data ? "TRUE" : "FALSE");
      });

    // Publisher
    std::string cmd_vel_topic;
    get_parameter("cmd_vel_topic", cmd_vel_topic);
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_topic, 1);

    // Control loop timer
    const auto period = std::chrono::duration<double>(1.0 / controller_frequency);
    control_timer_ = create_wall_timer(period, [this]() { controlLoop(); });

    RCLCPP_INFO(
      get_logger(),
      "AckermannMPPINode configured. Control loop: %.1f Hz, plan timeout: %.1f s",
      controller_frequency, plan_timeout_s_);
  }

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> getCostmapROS()
  {
    return costmap_ros_;
  }

  void deactivate()
  {
    control_timer_->cancel();
    optimizer_.shutdown();
    costmap_ros_->deactivate();
    costmap_ros_->cleanup();
  }

private:
  void controlLoop()
  {
    if (!nav_enabled_.load()) {
      RCLCPP_DEBUG(get_logger(), "Navigation disabled — sending zero velocity.");
      geometry_msgs::msg::TwistStamped zero;
      zero.header.stamp = now();
      zero.header.frame_id = base_frame_;
      cmd_vel_pub_->publish(zero);
      return;
    }

    // --- Snapshot shared state under locks ---
    nav_msgs::msg::Path plan;
    nav_msgs::msg::Odometry odom;
    rclcpp::Time plan_time;
    {
      std::lock_guard<std::mutex> lock(plan_mutex_);
      if (current_plan_.poses.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[mppi] nav enabled but no plan yet");
        return;
      }
      plan = current_plan_;
      plan_time = last_plan_time_;
    }
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      odom = last_odom_;
    }

    // --- Staleness check ---
    const double plan_age = (now() - plan_time).seconds();
    if (plan_age > plan_timeout_s_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Plan is stale (%.1f s old, timeout %.1f s) — sending zero velocity.",
        plan_age, plan_timeout_s_);
      geometry_msgs::msg::TwistStamped zero;
      zero.header.stamp = now();
      zero.header.frame_id = base_frame_;
      cmd_vel_pub_->publish(zero);
      optimizer_.reset();
      return;
    }

    // --- Look up robot pose in the global frame ---
    geometry_msgs::msg::PoseStamped robot_pose;
    robot_pose.header.frame_id = global_frame_;
    robot_pose.header.stamp = now();

    try {
      auto transform = tf_->lookupTransform(
        global_frame_, base_frame_,
        tf2::TimePointZero);
      robot_pose.pose.position.x = transform.transform.translation.x;
      robot_pose.pose.position.y = transform.transform.translation.y;
      robot_pose.pose.position.z = transform.transform.translation.z;
      robot_pose.pose.orientation = transform.transform.rotation;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Could not get robot pose: %s", ex.what());
      return;
    }

    // Robot speed from odometry
    geometry_msgs::msg::Twist robot_speed;
    robot_speed.linear.x = odom.twist.twist.linear.x;
    robot_speed.linear.y = odom.twist.twist.linear.y;
    robot_speed.angular.z = odom.twist.twist.angular.z;

    // Use the last pose of the plan as the goal
    const auto & goal = plan.poses.back().pose;

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "[mppi] running evalControl — pose=(%.2f, %.2f) plan_poses=%zu",
      robot_pose.pose.position.x, robot_pose.pose.position.y, plan.poses.size());

    try {
      auto [cmd, optimal_traj] = optimizer_.evalControl(
        robot_pose, robot_speed, plan, goal);
      geometry_msgs::msg::TwistStamped stamped;
      stamped.header.stamp = now();
      stamped.header.frame_id = base_frame_;
      stamped.twist = cmd.twist;
      cmd_vel_pub_->publish(stamped);
    } catch (const std::runtime_error & ex) {
      RCLCPP_ERROR(get_logger(), "MPPI failed: %s — sending zero velocity.", ex.what());
      geometry_msgs::msg::TwistStamped zero;
      zero.header.stamp = now();
      zero.header.frame_id = base_frame_;
      cmd_vel_pub_->publish(zero);
      optimizer_.reset();
    }
  }

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Costmap (sensor integration + inflation)
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  // MPPI optimizer
  mppi::Optimizer optimizer_;

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr nav_enabled_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::atomic<bool> nav_enabled_{true};

  // Shared state — protected by corresponding mutexes.
  // Written by subscription callbacks, read by control timer; all run in a
  // MultiThreadedExecutor so concurrent access is real.
  std::mutex plan_mutex_;
  std::mutex odom_mutex_;
  nav_msgs::msg::Path current_plan_;
  nav_msgs::msg::Odometry last_odom_;
  rclcpp::Time last_plan_time_{0, 0, RCL_ROS_TIME};

  // Cached parameters (never change after configure())
  std::string global_frame_{"map"};
  std::string base_frame_{"base_link"};
  double plan_timeout_s_{2.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<AckermannMPPINode>();
  node->configure();

  // Use a multi-threaded executor so the Costmap2DROS lifecycle node
  // (which has its own callbacks for sensor data) runs alongside the control loop.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(node->getCostmapROS()->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
