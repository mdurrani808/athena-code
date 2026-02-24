#include "localizer/zed_tf_publisher.h"

#include <tf2/exceptions.h>

namespace localizer
{

    ZedTfPublisher::ZedTfPublisher()
        : Node("zed_tf_publisher")
    {
        declare_parameter<std::string>("pose_topic", "/zed/zed_node/pose");
        declare_parameter<std::string>("odom_topic", "/zed/zed_node/odom");
        declare_parameter<std::string>("map_frame", "map");
        declare_parameter<std::string>("odom_frame", "odom");
        declare_parameter<std::string>("base_frame", "base_footprint");
        declare_parameter<std::string>("camera_frame", "zed_camera_center");
        declare_parameter<bool>("publish_map_tf", true);

        pose_topic_ = get_parameter("pose_topic").as_string();
        odom_topic_ = get_parameter("odom_topic").as_string();
        map_frame_ = get_parameter("map_frame").as_string();
        odom_frame_ = get_parameter("odom_frame").as_string();
        base_frame_ = get_parameter("base_frame").as_string();
        camera_frame_ = get_parameter("camera_frame").as_string();
        publish_map_tf_ = get_parameter("publish_map_tf").as_bool();

        RCLCPP_INFO(get_logger(), "pose_topic    : %s", pose_topic_.c_str());
        RCLCPP_INFO(get_logger(), "odom_topic    : %s", odom_topic_.c_str());
        RCLCPP_INFO(get_logger(), "map_frame     : %s", map_frame_.c_str());
        RCLCPP_INFO(get_logger(), "odom_frame    : %s", odom_frame_.c_str());
        RCLCPP_INFO(get_logger(), "base_frame    : %s", base_frame_.c_str());
        RCLCPP_INFO(get_logger(), "camera_frame  : %s", camera_frame_.c_str());
        RCLCPP_INFO(get_logger(), "publish_map_tf: %s", publish_map_tf_ ? "true" : "false");

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


        auto sensor_qos = rclcpp::SensorDataQoS();

        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic_,
            sensor_qos,
            std::bind(&ZedTfPublisher::pose_callback, this, std::placeholders::_1));

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_,
            sensor_qos,
            std::bind(&ZedTfPublisher::odom_callback, this, std::placeholders::_1));

        init_timer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&ZedTfPublisher::init_camera_base_transform, this));

        RCLCPP_INFO(get_logger(),
                    "Waiting for static transform %s -> %s ...",
                    camera_frame_.c_str(), base_frame_.c_str());
    }

    void ZedTfPublisher::init_camera_base_transform()
    {
        try
        {
            geometry_msgs::msg::TransformStamped tf_camera_base =
                tf_buffer_->lookupTransform(
                    camera_frame_,
                    base_frame_,
                    tf2::TimePointZero);

            tf2::Transform T;
            tf2::fromMsg(tf_camera_base.transform, T);
            T_camera_base_ = T;

            init_timer_->cancel();
            RCLCPP_INFO(get_logger(),
                        "Cached static transform %s -> %s. Node ready.",
                        camera_frame_.c_str(), base_frame_.c_str());
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(get_logger(),
                        "init: waiting for %s -> %s: %s",
                        camera_frame_.c_str(), base_frame_.c_str(), ex.what());
        }
    }


    void ZedTfPublisher::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!T_camera_base_.has_value())
        {
            return; // static transform not yet available
        }

        // The ZED odom message gives T_odom_camera (pose of the camera in odom).
        // Compose with the cached T_camera_base to get T_odom_base:
        //   T_odom_base = T_odom_camera * T_camera_base
        tf2::Transform T_odom_camera;
        tf2::fromMsg(msg->pose.pose, T_odom_camera);

        tf2::Transform T_odom_base = T_odom_camera * (*T_camera_base_);

        // Build the corrected odom -> base_footprint transform.
        geometry_msgs::msg::TransformStamped tf_odom_base;
        tf_odom_base.header.stamp = msg->header.stamp;
        tf_odom_base.header.frame_id = odom_frame_;
        tf_odom_base.child_frame_id = base_frame_;

        const tf2::Vector3 &o = T_odom_base.getOrigin();
        tf2::Quaternion q = T_odom_base.getRotation();

        tf_odom_base.transform.translation.x = o.x();
        tf_odom_base.transform.translation.y = o.y();
        tf_odom_base.transform.translation.z = o.z();
        tf_odom_base.transform.rotation.x = q.x();
        tf_odom_base.transform.rotation.y = q.y();
        tf_odom_base.transform.rotation.z = q.z();
        tf_odom_base.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(tf_odom_base);

        {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_odom_base_ = tf_odom_base;
        }

        if (publish_map_tf_)
        {
            try_publish_map_to_odom(msg->header.stamp);
        }
    }

    void ZedTfPublisher::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!T_camera_base_.has_value())
        {
            return; // static transform not yet available
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_pose_ = *msg;
        }

        if (publish_map_tf_)
        {
            try_publish_map_to_odom(msg->header.stamp);
        }
    }

    void ZedTfPublisher::try_publish_map_to_odom(const rclcpp::Time &stamp)
    {
        geometry_msgs::msg::PoseStamped pose;
        geometry_msgs::msg::TransformStamped odom_base;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!latest_pose_.has_value() || !latest_odom_base_.has_value())
            {
                return;
            }
            pose = *latest_pose_;
            odom_base = *latest_odom_base_;
        }

        tf2::Transform T_map_camera;
        tf2::fromMsg(pose.pose, T_map_camera);

        tf2::Transform T_map_base = T_map_camera * (*T_camera_base_);

        tf2::Transform T_odom_base;
        tf2::fromMsg(odom_base.transform, T_odom_base);

        tf2::Transform T_map_odom = T_map_base * T_odom_base.inverse();

        geometry_msgs::msg::TransformStamped tf_map_odom;
        tf_map_odom.header.stamp = stamp;
        tf_map_odom.header.frame_id = map_frame_;
        tf_map_odom.child_frame_id = odom_frame_;

        const tf2::Vector3 &origin = T_map_odom.getOrigin();
        tf2::Quaternion rot = T_map_odom.getRotation();

        tf_map_odom.transform.translation.x = origin.x();
        tf_map_odom.transform.translation.y = origin.y();
        tf_map_odom.transform.translation.z = origin.z();
        tf_map_odom.transform.rotation.x = rot.x();
        tf_map_odom.transform.rotation.y = rot.y();
        tf_map_odom.transform.rotation.z = rot.z();
        tf_map_odom.transform.rotation.w = rot.w();

        tf_broadcaster_->sendTransform(tf_map_odom);
    }

} // namespace localizer

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<localizer::ZedTfPublisher>());
    rclcpp::shutdown();
    return 0;
}
