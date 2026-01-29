#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float64.hpp"
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

using namespace std::chrono_literals;
using namespace mavsdk;

class PixhawkNode : public rclcpp::Node
{
public:
    PixhawkNode()
    : Node("athena_gps"), last_gps_info_{}, last_attitude_quaternion_{}, last_heading_deg_{}
    {
        this->declare_parameter("usb_path", "/dev/ttyACM0");
        this->declare_parameter("imu_rate", 10.0);
        this->declare_parameter("gps_info_rate", 1.0);
        this->declare_parameter("attitude_rate", 10.0);
        this->declare_parameter("imu_frame_id", "imu_link");
        this->declare_parameter("gps_frame_id", "gps_link");
        this->declare_parameter("imu_topic", "imu/data");
        this->declare_parameter("gps_topic", "gps/fix");
        this->declare_parameter("heading_topic", "heading");
        this->declare_parameter("mavsdk_system_id", 10);
        this->declare_parameter("mavsdk_component_id", 1);
        this->declare_parameter("mavsdk_always_send_heartbeats", false);

        std::string usb_path = this->get_parameter("usb_path").as_string();
        double imu_rate = this->get_parameter("imu_rate").as_double();
        double gps_info_rate = this->get_parameter("gps_info_rate").as_double();
        double attitude_rate = this->get_parameter("attitude_rate").as_double();
        imu_frame_id_ = this->get_parameter("imu_frame_id").as_string();
        gps_frame_id_ = this->get_parameter("gps_frame_id").as_string();
        std::string imu_topic = this->get_parameter("imu_topic").as_string();
        std::string gps_topic = this->get_parameter("gps_topic").as_string();
        std::string heading_topic = this->get_parameter("heading_topic").as_string();
        int mavsdk_system_id = this->get_parameter("mavsdk_system_id").as_int();
        int mavsdk_component_id = this->get_parameter("mavsdk_component_id").as_int();
        bool mavsdk_always_send_heartbeats = this->get_parameter("mavsdk_always_send_heartbeats").as_bool();

        mavsdk_ = std::make_unique<Mavsdk>(Mavsdk::Configuration{
            static_cast<uint8_t>(mavsdk_system_id),
            static_cast<uint8_t>(mavsdk_component_id),
            mavsdk_always_send_heartbeats
        });

        ConnectionResult conn_result = mavsdk_->add_any_connection(usb_path);
        if (conn_result != ConnectionResult::Success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to Pixhawk at %s", usb_path.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Connected to Pixhawk at %s", usb_path.c_str());

        while (mavsdk_->systems().empty()) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for system connection...");
            rclcpp::sleep_for(std::chrono::seconds(1));
        }

        auto system = mavsdk_->systems().at(0);
        telemetry_ = std::make_shared<Telemetry>(system);

        Telemetry::Result imu_result = telemetry_->set_rate_imu(imu_rate);
        Telemetry::Result gps_info_result = telemetry_->set_rate_gps_info(gps_info_rate);
        Telemetry::Result attitude_result = telemetry_->set_rate_attitude_quaternion(attitude_rate);

        if (imu_result != Telemetry::Result::Success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set IMU rate to %.1f Hz", imu_rate);
            return;
        }

        if (gps_info_result != Telemetry::Result::Success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set GPS info rate to %.1f Hz", gps_info_rate);
            return;
        }

        if (attitude_result != Telemetry::Result::Success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set attitude rate to %.1f Hz", attitude_rate);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Telemetry rates configured successfully");

        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic, 10);
        gps_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(gps_topic, 10);
        heading_publisher_ = this->create_publisher<std_msgs::msg::Float64>(heading_topic, 10);

        telemetry_->subscribe_imu([this](const Telemetry::Imu &imu) {
            imu_callback(imu);
        });

        telemetry_->subscribe_attitude_quaternion([this](const Telemetry::Quaternion &quaternion) {
            last_attitude_quaternion_ = quaternion;
        });

        telemetry_->subscribe_heading([this](Telemetry::Heading heading) {
            last_heading_deg_ = heading.heading_deg;
            publish_heading();
        });

        telemetry_->subscribe_raw_gps([this](const Telemetry::RawGps &raw_gps) {
            raw_gps_callback(raw_gps);
        });

        telemetry_->subscribe_gps_info([this](const Telemetry::GpsInfo &gps_info) {
            last_gps_info_ = gps_info;
        });

        RCLCPP_INFO(this->get_logger(), "Pixhawk GPS node initialized successfully");
    }

private:
    std::unique_ptr<Mavsdk> mavsdk_;
    std::shared_ptr<Telemetry> telemetry_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_publisher_;

    Telemetry::GpsInfo last_gps_info_;
    Telemetry::Quaternion last_attitude_quaternion_;
    double last_heading_deg_;

    std::string imu_frame_id_;
    std::string gps_frame_id_;

    void publish_heading() {
        auto msg = std_msgs::msg::Float64();
        msg.data = last_heading_deg_;
        heading_publisher_->publish(msg);
    }

    /**
     * convert quaternion from FRD (Forward-Right-Down) to FLU (Forward-Left-Up) frame.
     */
    void quaternion_frd_to_flu(const Telemetry::Quaternion &frd,
                               double &w_out, double &x_out, double &y_out, double &z_out) {
        w_out = -frd.x;
        x_out = frd.w;
        y_out = -frd.z;
        z_out = frd.y;
    }

    void imu_callback(const Telemetry::Imu imu) {
        auto msg = sensor_msgs::msg::Imu();
        msg.header.stamp = this->now();
        msg.header.frame_id = imu_frame_id_;

        quaternion_frd_to_flu(last_attitude_quaternion_,
                             msg.orientation.w,
                             msg.orientation.x,
                             msg.orientation.y,
                             msg.orientation.z);

        // convert angular velocity from FRD to FLU frame
        // FLU: X=forward, Y=left, Z=up
        // FRD: X=forward, Y=right, Z=down
        msg.angular_velocity.x = imu.angular_velocity_frd.forward_rad_s;
        msg.angular_velocity.y = -imu.angular_velocity_frd.right_rad_s;
        msg.angular_velocity.z = -imu.angular_velocity_frd.down_rad_s;

        msg.linear_acceleration.x = imu.acceleration_frd.forward_m_s2;
        msg.linear_acceleration.y = -imu.acceleration_frd.right_m_s2;
        msg.linear_acceleration.z = -imu.acceleration_frd.down_m_s2;

        bool has_orientation = (last_attitude_quaternion_.w != 0.0 ||
                               last_attitude_quaternion_.x != 0.0 ||
                               last_attitude_quaternion_.y != 0.0 ||
                               last_attitude_quaternion_.z != 0.0);

        for (int i = 0; i < 9; i++) {
            msg.angular_velocity_covariance[i] = 0.0;
            msg.linear_acceleration_covariance[i] = 0.0;
            msg.orientation_covariance[i] = has_orientation ? 0.0 : -1.0;
        }

        imu_publisher_->publish(msg);
    }

    void raw_gps_callback(const Telemetry::RawGps raw_gps) {
        auto msg = sensor_msgs::msg::NavSatFix();

        // convert microsections to nanoseconds for ros time
        auto unix_epoch_time = std::chrono::microseconds(raw_gps.timestamp_us);
        auto ros_time = rclcpp::Time(unix_epoch_time.count() * 1000);
        msg.header.stamp = ros_time;
        msg.header.frame_id = gps_frame_id_;

        msg.latitude = raw_gps.latitude_deg;
        msg.longitude = raw_gps.longitude_deg;
        msg.altitude = raw_gps.altitude_ellipsoid_m;

        // Map GPS fix to ROS fix 
        if (last_gps_info_.num_satellites > 0) {
            switch (last_gps_info_.fix_type) {
                case Telemetry::FixType::NoGps:
                case Telemetry::FixType::NoFix:
                    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
                    break;
                case Telemetry::FixType::Fix2D:
                    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
                    break;
                case Telemetry::FixType::Fix3D:
                case Telemetry::FixType::FixDgps:
                case Telemetry::FixType::RtkFloat:
                case Telemetry::FixType::RtkFixed:
                    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
                    break;
                default:
                    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
            }
        } else {
            msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        }

        msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

        // calcilate variances
        if (!std::isnan(raw_gps.horizontal_uncertainty_m) &&
            !std::isnan(raw_gps.vertical_uncertainty_m)) {

            double h_variance = raw_gps.horizontal_uncertainty_m * raw_gps.horizontal_uncertainty_m;
            double v_variance = raw_gps.vertical_uncertainty_m * raw_gps.vertical_uncertainty_m;

            msg.position_covariance[0] = h_variance;
            msg.position_covariance[4] = h_variance;
            msg.position_covariance[8] = v_variance;

            // refine estimate using doppler
            if (!std::isnan(raw_gps.hdop) && !std::isnan(raw_gps.vdop)) {
                msg.position_covariance[0] = h_variance * (raw_gps.hdop / 2.0);
                msg.position_covariance[4] = h_variance * (raw_gps.hdop / 2.0);
                msg.position_covariance[8] = v_variance * raw_gps.vdop;
            }

            msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        } else {
            msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
        }

        gps_publisher_->publish(msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PixhawkNode>());
    rclcpp::shutdown();
    return 0;
}