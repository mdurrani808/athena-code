#include <rclcpp/rclcpp.hpp>
#include <athena_localizer/state_estimator.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

class AthenaLocalizerTestNode : public rclcpp::Node
{
public:
  AthenaLocalizerTestNode()
  : Node("athena_localizer_test_node")
  {
    RCLCPP_INFO(this->get_logger(), "Starting Athena Localizer Test Node");

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/kitti/oxts/imu", 10,
      std::bind(&AthenaLocalizerTestNode::imu_callback, this, std::placeholders::_1));

    gnss_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/kitti/oxts/gps/fix", 10,
      std::bind(&AthenaLocalizerTestNode::gnss_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&AthenaLocalizerTestNode::odom_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&AthenaLocalizerTestNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Subscribers created");
  }

  void initialize_state_estimator()
  {
    // Initialize state estimator
    athena_localizer::StateEstimatorParams params;
    params.imu_accel_noise = 0.1;
    params.imu_gyro_noise = 0.01;
    params.imu_bias_noise = 0.001;
    params.gnss_noise = 1.0;
    params.odom_noise = 0.1;
    params.max_time_window = 10.0;
    params.max_states = 100;

    // Get tf_prefix parameter
    params.frames.tf_prefix = this->declare_parameter("tf_prefix", std::string("test_jawn"));

    state_estimator_.initialize(params, shared_from_this());
    RCLCPP_INFO(this->get_logger(), "State estimator initialized");
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    RCLCPP_DEBUG(this->get_logger(), "Received IMU data");
    state_estimator_.fuse_imu(msg);
  }

  void gnss_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    RCLCPP_INFO(
      this->get_logger(), "Received GNSS data: lat=%.6f, lon=%.6f",
      msg->latitude, msg->longitude);
    state_estimator_.fuse_gnss(msg);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    RCLCPP_DEBUG(this->get_logger(), "Received odom data");
    state_estimator_.fuse_odometry(msg);
  }

  void timer_callback()
  {    auto latest_state = state_estimator_.get_latest_state();
    if (latest_state.has_value()) {
      RCLCPP_DEBUG(this->get_logger(), "State estimator has valid state");
      state_estimator_.publish_transforms();
    } else {
      RCLCPP_DEBUG(this->get_logger(), "State estimator not yet initialized");
    }
  }

  athena_localizer::StateEstimator state_estimator_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr debug_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<AthenaLocalizerTestNode>();

  node->initialize_state_estimator();

  RCLCPP_INFO(rclcpp::get_logger("main"), "Spinning Athena Localizer Test Node");
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
