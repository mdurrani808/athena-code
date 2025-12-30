#pragma once

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// GTSAM
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>

// std
#include <mutex>
#include <optional>
#include <deque>
#include <memory>

namespace localizer
{

struct FrameParams
{
    std::string tf_prefix = "";
    std::string map_frame = "map";
    std::string odom_frame = "odom";
    std::string base_frame = "base_link";
    std::string imu_frame = "imu_link";
    std::string gnss_frame = "gnss_link";
};

struct StateEstimatorParams
{
    // Noise parameters
    double imu_accel_noise = 0.1;     // m/s^2
    double imu_gyro_noise = 0.005;    // rad/s
    double imu_bias_noise = 0.0001;   // bias random walk
    double gnss_noise = 1.0;          // meters
    double odom_noise = 0.1;          // meters

    // Window parameters
    double max_time_window = 10.0;    // seconds
    size_t max_states = 100;

    // Frame names
    FrameParams frames;
};

struct EstimatedState
{
    rclcpp::Time timestamp;
    gtsam::NavState nav_state;
    gtsam::imuBias::ConstantBias imu_bias;
    Eigen::Matrix<double, 15, 15> covariance;

    nav_msgs::msg::Odometry to_odometry(const std::string& frame_id, const std::string& child_frame_id) const;
};

class StateEstimator : public rclcpp::Node
{
public:
    StateEstimator();
    ~StateEstimator();

    void reset();

    std::optional<EstimatedState> get_latest_state() const;
    std::optional<EstimatedState> get_state_at_time(const rclcpp::Time& timestamp) const;

    bool is_initialized() const { return initialized_; }

private:
    // Sensor fusion (used as subscription callbacks)
    void fuse_imu(sensor_msgs::msg::Imu::SharedPtr imu_msg);
    void fuse_gnss(sensor_msgs::msg::NavSatFix::SharedPtr gnss_msg);
    void fuse_odometry(nav_msgs::msg::Odometry::SharedPtr odom_msg);

    // Publishing
    void publish_state();
    void publish_transforms();

    // Graph optimization
    void optimize_graph();
    void add_imu_factor();
    void add_gnss_factor(const sensor_msgs::msg::NavSatFix::SharedPtr& gnss_msg);
    void add_odom_factor(const nav_msgs::msg::Odometry::SharedPtr& odom_msg);
    void createNewState();
    void marginalize_old_states();
    void initialize_with_gnss(const gtsam::Point3& gnss_pos, const rclcpp::Time& timestamp);

    // Coordinate conversion
    gtsam::Point3 lla_to_enu(double lat, double lon, double alt) const;
    void set_enu_origin(double lat, double lon, double alt);

    // Key generation
    gtsam::Key pose_key(size_t i) const { return gtsam::Symbol('x', i); }
    gtsam::Key vel_key(size_t i) const { return gtsam::Symbol('v', i); }
    gtsam::Key bias_key(size_t i) const { return gtsam::Symbol('b', i); }

    // Parameters
    StateEstimatorParams params_;

    // ROS2 subscriptions and publishers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    // TF2 components
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr tf_timer_;

    // Transform caches
    std::optional<gtsam::Pose3> imu_to_base_;
    std::optional<gtsam::Point3> gnss_to_base_;

    // GTSAM components
    std::unique_ptr<gtsam::ISAM2> isam_;
    std::unique_ptr<gtsam::NonlinearFactorGraph> new_factors_;
    std::unique_ptr<gtsam::Values> new_values_;
    std::unique_ptr<gtsam::PreintegratedImuMeasurements> imu_preintegrated_;

    // State management
    std::deque<EstimatedState> state_history_;
    std::deque<sensor_msgs::msg::Imu> imu_buffer_;

    // State tracking
    gtsam::Pose3 odom_to_base_;
    gtsam::Pose3 map_to_odom_;
    gtsam::Key current_pose_key_, current_vel_key_, current_bias_key_;
    size_t key_index_;
    bool initialized_;
    rclcpp::Time last_optimization_time_;
    std::optional<gtsam::Pose3> prev_odom_pose_;

    // ENU origin
    bool enu_origin_set_;
    double origin_lat_, origin_lon_, origin_alt_;

    // Noise models
    gtsam::SharedNoiseModel imu_noise_model_;
    gtsam::SharedNoiseModel gnss_noise_model_;
    gtsam::SharedNoiseModel odom_noise_model_;

    mutable std::mutex state_mutex_;
};

} // namespace localizer