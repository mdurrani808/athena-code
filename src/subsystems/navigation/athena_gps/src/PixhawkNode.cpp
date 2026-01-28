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
    : Node("minimal_publisher"), last_gps_info_{}, last_attitude_quaternion_{}, last_heading_deg_{}
    {
        mavsdk_ = std::make_unique<Mavsdk>(Mavsdk::Configuration{10, 1, false});
        // this must be configured in the launch file
        this->declare_parameter("usb_path", "/dev/ttyACM0");
        ConnectionResult conn_result = mavsdk_->add_any_connection(this->get_parameter("usb_path").as_string());
        if (conn_result != ConnectionResult::Success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to Pixhawk! Please check physical connection.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Connection to pixhawk successful.");

        while (mavsdk_->systems().empty()) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for system connection...");
            rclcpp::sleep_for(std::chrono::seconds(1));
        }

        auto system = mavsdk_->systems().at(0);
        telemetry_ = std::make_shared<Telemetry>(system);

        Telemetry::Result imu_result = telemetry_->set_rate_imu(10.0);
        Telemetry::Result gps_info_result = telemetry_->set_rate_gps_info(1.0);
        Telemetry::Result attitude_result = telemetry_->set_rate_attitude_quaternion(10.0);

        if (imu_result != Telemetry::Result::Success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set IMU rate");
            return;
        }

        if (gps_info_result != Telemetry::Result::Success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set GPS info rate");
            return;
        }

        if (attitude_result != Telemetry::Result::Success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set attitude quaternion rate");
            return;
        }

        // Create publishers
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
        gps_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", 10);
        heading_publisher_ = this->create_publisher<std_msgs::msg::Float64>("heading", 10);

        // Subscribe to IMU data
        telemetry_->subscribe_imu([this](const Telemetry::Imu &imu) {
            imu_callback(imu);
        });

        // Subscribe to attitude quaternion
        telemetry_->subscribe_attitude_quaternion([this](const Telemetry::Quaternion &quaternion) {
            last_attitude_quaternion_ = quaternion;
        });

        // Subscribe to heading
        telemetry_->subscribe_heading([this](Telemetry::Heading heading) {
            last_heading_deg_ = heading.heading_deg;
            publish_heading();
        });

        // Subscribe to raw GPS
        telemetry_->subscribe_raw_gps([this](const Telemetry::RawGps &raw_gps) {
            raw_gps_callback(raw_gps);
        });

        // Subscribe to GPS info for fix type
        telemetry_->subscribe_gps_info([this](const Telemetry::GpsInfo &gps_info) {
            last_gps_info_ = gps_info;
        });
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

    void publish_heading() {
        auto msg = std_msgs::msg::Float64();
        msg.data = last_heading_deg_;
        heading_publisher_->publish(msg);
    }

    void imu_callback(const Telemetry::Imu imu) {
        auto msg = sensor_msgs::msg::Imu();
        msg.header.stamp = this->now();
        msg.header.frame_id = "imu_link";

        // Use the full quaternion from attitude
        msg.orientation.w = last_attitude_quaternion_.w;
        msg.orientation.x = last_attitude_quaternion_.x;
        msg.orientation.y = last_attitude_quaternion_.y;
        msg.orientation.z = last_attitude_quaternion_.z;

        msg.angular_velocity.x = imu.angular_velocity_frd.forward_rad_s;
        msg.angular_velocity.y = imu.angular_velocity_frd.right_rad_s;
        msg.angular_velocity.z = imu.angular_velocity_frd.down_rad_s;

        msg.linear_acceleration.x = imu.acceleration_frd.forward_m_s2;
        msg.linear_acceleration.y = imu.acceleration_frd.right_m_s2;
        msg.linear_acceleration.z = imu.acceleration_frd.down_m_s2;

        // Set covariances
        for(int i = 0; i < 9; i++) {
            msg.angular_velocity_covariance[i] = 0.0;
            msg.linear_acceleration_covariance[i] = 0.0;
            // Only set orientation covariance to -1 if we haven't received quaternion data yet
            msg.orientation_covariance[i] = (last_attitude_quaternion_.w == 0.0 && 
                                          last_attitude_quaternion_.x == 0.0 && 
                                          last_attitude_quaternion_.y == 0.0 && 
                                          last_attitude_quaternion_.z == 0.0) ? -1 : 0.0;
        }

        imu_publisher_->publish(msg);
    }

    void raw_gps_callback(const Telemetry::RawGps raw_gps) {
        auto msg = sensor_msgs::msg::NavSatFix();

        auto unix_epoch_time = std::chrono::microseconds(raw_gps.timestamp_us);
        auto ros_time = rclcpp::Time(unix_epoch_time.count() * 1000); // Convert to nanoseconds

        msg.header.stamp = ros_time;
        msg.header.frame_id = "gps_link";

        msg.latitude = raw_gps.latitude_deg;
        msg.longitude = raw_gps.longitude_deg;
        msg.altitude = raw_gps.altitude_ellipsoid_m;  // Using ellipsoid altitude

        // Set the status based on GPS info
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

        // Set position covariance using actual uncertainties
        if (!std::isnan(raw_gps.horizontal_uncertainty_m) &&
            !std::isnan(raw_gps.vertical_uncertainty_m)) {
            // Convert uncertainties to variances
            double h_variance = raw_gps.horizontal_uncertainty_m * raw_gps.horizontal_uncertainty_m;
            double v_variance = raw_gps.vertical_uncertainty_m * raw_gps.vertical_uncertainty_m;

            msg.position_covariance[0] = h_variance; // xx
            msg.position_covariance[4] = h_variance; // yy (assuming isotropic horizontal uncertainty)
            msg.position_covariance[8] = v_variance; // zz

            // If we have HDOP and VDOP, we can make the horizontal covariance more accurate
            if (!std::isnan(raw_gps.hdop) && !std::isnan(raw_gps.vdop)) {
                msg.position_covariance[0] = h_variance * (raw_gps.hdop / 2.0); // xx
                msg.position_covariance[4] = h_variance * (raw_gps.hdop / 2.0); // yy
                msg.position_covariance[8] = v_variance * raw_gps.vdop;         // zz
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