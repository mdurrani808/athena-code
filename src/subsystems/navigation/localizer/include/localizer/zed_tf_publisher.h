#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <mutex>
#include <optional>
#include <string>

namespace localizer {

/**
 * @brief Publishes TF transforms derived from ZED camera pose and odometry.
 *
 * Subscribes to:
 *   - pose_topic  (geometry_msgs/PoseStamped): pose of the ZED camera frame in the map frame.
 *   - odom_topic  (nav_msgs/Odometry):         pose of the robot in the odom frame.
 *
 * The camera-to-base offset is resolved at runtime by looking up the transform
 * from camera_frame -> base_frame in the TF tree, so no hard-coded offsets are needed.
 *
 * Publishes (via TF broadcaster):
 *   - odom -> base_footprint : taken directly from the odometry message.
 *   - map  -> odom           : computed as T_map_base * inv(T_odom_base), where
 *                              T_map_base = T_map_camera * T_camera_base (TF lookup).
 *                              Can be disabled via 'publish_map_tf' in the YAML config.
 */
class ZedTfPublisher : public rclcpp::Node
{
public:
    ZedTfPublisher();

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @brief Attempts to compute and publish the map -> odom transform.
     *
     * Requires both latest_pose_ and latest_odom_ to be populated, and a valid
     * camera_frame -> base_frame transform to be available in the TF tree.
     *
     * @param stamp Timestamp to stamp the outgoing transform.
     */
    void try_publish_map_to_odom(const rclcpp::Time & stamp);

    /**
     * @brief Called on a timer until the camera_frame -> base_frame static
     *        transform is available. Caches it in T_camera_base_ and cancels
     *        the timer once successful.
     */
    void init_camera_base_transform();

    // Subscriptions
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // TF broadcaster and listener (used only during init for the static lookup)
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Initialization timer: retries the camera->base TF lookup until available.
    rclcpp::TimerBase::SharedPtr init_timer_;

    // Cached static transform: camera_frame -> base_frame.
    // Set once during initialization; callbacks return early until populated.
    std::optional<tf2::Transform> T_camera_base_;

    // Cached latest messages (mutex-protected).
    // latest_odom_base_ stores the already-corrected odom -> base_footprint
    // transform (T_odom_camera already composed with T_camera_base).
    std::mutex mutex_;
    std::optional<geometry_msgs::msg::PoseStamped> latest_pose_;
    std::optional<geometry_msgs::msg::TransformStamped> latest_odom_base_;

    // Configurable parameters
    std::string pose_topic_;
    std::string odom_topic_;
    std::string map_frame_;
    std::string odom_frame_;
    std::string base_frame_;
    std::string camera_frame_;
    bool publish_map_tf_;
};

}  // namespace localizer
