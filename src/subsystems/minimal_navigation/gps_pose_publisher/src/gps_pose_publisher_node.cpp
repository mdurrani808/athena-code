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

#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/string.hpp"
#include "msgs/msg/heading.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "msgs/srv/lat_lon_to_enu.hpp"

#include <GeographicLib/LocalCartesian.hpp>

class GpsPosePublisher : public rclcpp::Node
{
public:
  explicit GpsPosePublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("gps_pose_publisher", options)
  {
    declare_parameter("heading_topic", std::string("/gps/heading"));
    declare_parameter("use_start_gate_ref", false);
    declare_parameter("origin_lat", std::numeric_limits<double>::quiet_NaN());
    declare_parameter("origin_lon", std::numeric_limits<double>::quiet_NaN());
    declare_parameter("origin_alt", 0.0);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/robot_pose", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/gps_status", 10);

    fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/fix", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) { onFix(msg); });

    std::string heading_topic;
    get_parameter("heading_topic", heading_topic);
    heading_sub_ = create_subscription<msgs::msg::Heading>(
      heading_topic, rclcpp::SensorDataQoS(),
      [this](const msgs::msg::Heading::SharedPtr msg) {
        heading_enu_rad_ = (90.0 - msg->compass_bearing) * M_PI / 180.0;
        has_heading_ = true;
      });

    // Pre-set origin from params if provided (non-NaN overrides first-fix logic)
    const double param_lat = get_parameter("origin_lat").as_double();
    const double param_lon = get_parameter("origin_lon").as_double();
    const double param_alt = get_parameter("origin_alt").as_double();
    if (!std::isnan(param_lat) && !std::isnan(param_lon)) {
      setOrigin(param_lat, param_lon, param_alt);
    }

    bool use_start_gate_ref;
    get_parameter("use_start_gate_ref", use_start_gate_ref);
    if (use_start_gate_ref) {
      start_gate_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        "/start_gate_ref", 1,
        [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
          if (!origin_set_) {
            setOrigin(msg->latitude, msg->longitude, msg->altitude);
          }
        });
    }

    latlon_srv_ = create_service<msgs::srv::LatLonToENU>(
      "~/latlon_to_enu",
      [this](
        const msgs::srv::LatLonToENU::Request::SharedPtr req,
        msgs::srv::LatLonToENU::Response::SharedPtr res)
      {
        if (!origin_set_) {
          RCLCPP_ERROR(get_logger(), "latlon_to_enu called before origin is set");
          res->x = 0.0; res->y = 0.0; res->z = 0.0;
          return;
        }
        double x, y, z;
        projection_.Forward(req->lat, req->lon, 0.0, x, y, z);
        res->x = x; res->y = y; res->z = z;
      });
  }

private:
  void setOrigin(double lat, double lon, double alt)
  {
    projection_.Reset(lat, lon, alt);
    origin_set_ = true;
    RCLCPP_INFO(get_logger(), "GPS origin set: lat=%.8f lon=%.8f alt=%.3f", lat, lon, alt);
  }

  void onFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
      std_msgs::msg::String s;
      s.data = "NO_FIX";
      status_pub_->publish(s);
      return;
    }

    if (!origin_set_) {
      bool use_start_gate_ref;
      get_parameter("use_start_gate_ref", use_start_gate_ref);
      if (!use_start_gate_ref) {
        setOrigin(msg->latitude, msg->longitude, msg->altitude);
      } else {
        return;
      }
    }

    double x, y, z;
    projection_.Forward(msg->latitude, msg->longitude, msg->altitude, x, y, z);

    // Publish fix quality + heading source
    std::string quality;
    switch (msg->status.status) {
      case sensor_msgs::msg::NavSatStatus::STATUS_FIX:      quality = "GPS";  break;
      case sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX: quality = "SBAS"; break;
      case sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX: quality = "GBAS"; break;
      default:                                               quality = "FIX";  break;
    }
    std_msgs::msg::String s;
    s.data = "FIX quality=" + quality + " heading=" + (has_heading_ ? "compass" : "none");
    status_pub_->publish(s);

    if (!has_heading_) {
      return;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = msg->header.stamp;
    pose.header.frame_id = "map";
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;

    const double half = heading_enu_rad_ * 0.5;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = std::sin(half);
    pose.pose.orientation.w = std::cos(half);

    pose_pub_->publish(pose);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = msg->header.stamp;
    tf.header.frame_id = "map";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.translation.z = z;
    tf.transform.rotation = pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
  }

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Subscription<msgs::msg::Heading>::SharedPtr heading_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr start_gate_sub_;
  rclcpp::Service<msgs::srv::LatLonToENU>::SharedPtr latlon_srv_;

  GeographicLib::LocalCartesian projection_;
  bool origin_set_{false};
  bool has_heading_{false};
  double heading_enu_rad_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GpsPosePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
