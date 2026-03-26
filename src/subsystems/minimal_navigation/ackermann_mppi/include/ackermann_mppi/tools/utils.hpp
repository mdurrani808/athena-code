// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Open Navigation LLC
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

#ifndef ACKERMANN_MPPI__TOOLS__UTILS_HPP_
#define ACKERMANN_MPPI__TOOLS__UTILS_HPP_

#include <Eigen/Dense>

#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <vector>

#include "angles/angles.h"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/color_rgba.hpp"

#include "rclcpp/rclcpp.hpp"

#include "ackermann_mppi/models/optimizer_settings.hpp"
#include "ackermann_mppi/models/control_sequence.hpp"
#include "ackermann_mppi/models/path.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "ackermann_mppi/critic_data.hpp"

namespace mppi::utils
{

// Use constexpr instead of #define to avoid macro pollution and enable type checking.
inline constexpr float M_PIF   = 3.141592653589793238462643383279502884e+00F;
inline constexpr float M_PIF_2 = 1.5707963267948966e+00F;

inline geometry_msgs::msg::Pose createPose(double x, double y, double z)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  return pose;
}

inline geometry_msgs::msg::Vector3 createScale(double x, double y, double z)
{
  geometry_msgs::msg::Vector3 scale;
  scale.x = x;
  scale.y = y;
  scale.z = z;
  return scale;
}

inline std_msgs::msg::ColorRGBA createColor(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

inline visualization_msgs::msg::Marker createMarker(
  int id, const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Vector3 & scale,
  const std_msgs::msg::ColorRGBA & color, const std::string & frame_id, const std::string & ns)
{
  using visualization_msgs::msg::Marker;
  Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = rclcpp::Time(0, 0);
  marker.ns = ns;
  marker.id = id;
  marker.type = Marker::SPHERE;
  marker.action = Marker::ADD;
  marker.pose = pose;
  marker.scale = scale;
  marker.color = color;
  return marker;
}

inline geometry_msgs::msg::TwistStamped toTwistStamped(
  float vx, float wz, const builtin_interfaces::msg::Time & stamp, const std::string & frame)
{
  geometry_msgs::msg::TwistStamped twist;
  twist.header.frame_id = frame;
  twist.header.stamp = stamp;
  twist.twist.linear.x = vx;
  twist.twist.angular.z = wz;
  return twist;
}

/**
 * @brief Convert path to a tensor of (x, y, yaw) arrays
 */
inline models::Path toTensor(const nav_msgs::msg::Path & path)
{
  auto result = models::Path{};
  result.reset(path.poses.size());
  for (size_t i = 0; i < path.poses.size(); ++i) {
    result.x(i) = path.poses[i].pose.position.x;
    result.y(i) = path.poses[i].pose.position.y;
    result.yaws(i) = tf2::getYaw(path.poses[i].pose.orientation);
  }
  return result;
}

/**
 * @brief Get the last pose in the path tensor
 */
inline geometry_msgs::msg::Pose getLastPathPose(const models::Path & path)
{
  const unsigned int path_last_idx = path.x.size() - 1;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, path.yaws(path_last_idx));
  geometry_msgs::msg::Pose p;
  p.position.x = path.x(path_last_idx);
  p.position.y = path.y(path_last_idx);
  p.orientation = tf2::toMsg(q);
  return p;
}

template<typename T>
auto normalize_angles(const T & angles)
{
  return (angles + M_PIF).unaryExpr(
    [&](const float x) {
      float remainder = std::fmod(x, 2.0f * M_PIF);
      return remainder < 0.0f ? remainder + M_PIF : remainder - M_PIF;
    });
}

template<typename F, typename T>
auto shortest_angular_distance(const F & from, const T & to)
{
  return normalize_angles(to - from);
}

/**
 * @brief Find the path point index furthest along the path that any trajectory in the batch
 * reaches (by closest-point mapping). Used by PathFollow and PathAlign critics.
 *
 * Vectorized: outer loop iterates over path points (n_path), inner computation is a
 * batch-wide Eigen array op over all trajectories simultaneously. This is significantly
 * faster than the naive scalar O(n_batch × n_path) nested loop because each iteration
 * of the outer loop operates on a [batch_size] SIMD-vectorizable array.
 */
inline size_t findPathFurthestReachedPoint(const CriticData & data)
{
  const int last_col = data.trajectories.x.cols() - 1;
  // End-positions of all trajectories: [batch_size] arrays.
  const auto traj_x = data.trajectories.x.col(last_col);
  const auto traj_y = data.trajectories.y.col(last_col);

  const Eigen::Index n_batch = traj_x.rows();
  const Eigen::Index n_path  = static_cast<Eigen::Index>(data.path.x.size());

  // Per-trajectory: closest path index found so far and its squared distance.
  Eigen::ArrayXf min_dist2 = Eigen::ArrayXf::Constant(n_batch, std::numeric_limits<float>::max());
  Eigen::ArrayXi best_idx  = Eigen::ArrayXi::Zero(n_batch);

  for (Eigen::Index j = 0; j != n_path; ++j) {
    // Vectorized over all batch_size trajectories at once.
    const Eigen::ArrayXf dx = traj_x - data.path.x(j);
    const Eigen::ArrayXf dy = traj_y - data.path.y(j);
    const Eigen::ArrayXf d2 = dx * dx + dy * dy;

    const auto better = (d2 < min_dist2).eval();
    min_dist2 = better.select(d2, min_dist2);
    best_idx  = better.select(Eigen::ArrayXi::Constant(n_batch, static_cast<int>(j)), best_idx);

    // Early exit: already found a trajectory that reaches the last path point.
    if (best_idx.maxCoeff() == static_cast<int>(n_path) - 1) {
      break;
    }
  }

  return static_cast<size_t>(best_idx.maxCoeff());
}

inline void setPathFurthestPointIfNotSet(CriticData & data)
{
  if (!data.furthest_reached_path_point) {
    data.furthest_reached_path_point = findPathFurthestReachedPoint(data);
  }
}

/**
 * @brief Check which path points are collision-free according to the costmap.
 * Stores results in data.path_pts_valid for reuse across critics.
 */
inline void findPathCosts(
  CriticData & data,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto * costmap = costmap_ros->getCostmap();
  unsigned int map_x, map_y;
  const size_t path_segments_count = data.path.x.size() - 1;
  data.path_pts_valid = std::vector<bool>(path_segments_count, false);
  const bool tracking_unknown = costmap_ros->getLayeredCostmap()->isTrackingUnknown();

  for (unsigned int idx = 0; idx < path_segments_count; idx++) {
    if (!costmap->worldToMap(data.path.x(idx), data.path.y(idx), map_x, map_y)) {
      (*data.path_pts_valid)[idx] = false;
      continue;
    }
    switch (costmap->getCost(map_x, map_y)) {
      case (nav2_costmap_2d::LETHAL_OBSTACLE):
        (*data.path_pts_valid)[idx] = false;
        continue;
      case (nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE):
        (*data.path_pts_valid)[idx] = false;
        continue;
      case (nav2_costmap_2d::NO_INFORMATION):
        (*data.path_pts_valid)[idx] = tracking_unknown ? true : false;
        continue;
    }
    (*data.path_pts_valid)[idx] = true;
  }
}

inline void setPathCostsIfNotSet(
  CriticData & data,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  if (!data.path_pts_valid) {
    findPathCosts(data, costmap_ros);
  }
}

inline float posePointAngle(
  const geometry_msgs::msg::Pose & pose, double point_x, double point_y, bool forward_preference)
{
  float pose_x = pose.position.x;
  float pose_y = pose.position.y;
  float pose_yaw = tf2::getYaw(pose.orientation);
  float yaw = atan2f(point_y - pose_y, point_x - pose_x);

  if (!forward_preference) {
    return std::min(
      fabs(angles::shortest_angular_distance(yaw, pose_yaw)),
      fabs(angles::shortest_angular_distance(yaw, angles::normalize_angle(pose_yaw + M_PIF))));
  }
  return fabs(angles::shortest_angular_distance(yaw, pose_yaw));
}

/**
 * @brief Apply Savitzky-Golay smoothing filter to the optimal control sequence.
 * Uses a 9-point quadratic filter to smooth sharp changes between control steps.
 */
inline void savitskyGolayFilter(
  models::ControlSequence & control_sequence,
  std::array<mppi::models::Control, 4> & control_history,
  const models::OptimizerSettings & settings)
{
  // Standard 9-point quadratic Savitzky-Golay smoothing coefficients (window = ±4).
  // Denominator 231 is the normalization factor for this window/polynomial order.
  // Reference: Savitzky & Golay, Analytical Chemistry, 1964, Table I (m=4, n=2).
  Eigen::Array<float, 9, 1> filter = {
    -21.0f, 14.0f, 39.0f, 54.0f, 59.0f, 54.0f, 39.0f, 14.0f, -21.0f};
  filter /= 231.0f;

  const unsigned int num_sequences = control_sequence.vx.size() - 1;
  // Skip smoothing for very short horizons: the 9-point window needs at least
  // 5 look-ahead points (indices 0..4), so sequences shorter than 20 provide
  // too little context for meaningful smoothing.
  if (num_sequences < 20) {return;}

  auto applyFilter = [&](const Eigen::Array<float, 9, 1> & data) -> float {
      return (data * filter).eval().sum();
    };

  auto applyFilterOverAxis =
    [&](Eigen::ArrayXf & sequence, const Eigen::ArrayXf & initial_sequence,
    const float hist_0, const float hist_1, const float hist_2, const float hist_3) -> void
    {
      float pt_m4 = hist_0, pt_m3 = hist_1, pt_m2 = hist_2, pt_m1 = hist_3;
      float pt = initial_sequence(0);
      float pt_p1 = initial_sequence(1), pt_p2 = initial_sequence(2);
      float pt_p3 = initial_sequence(3), pt_p4 = initial_sequence(4);

      for (unsigned int idx = 0; idx != num_sequences; idx++) {
        sequence(idx) = applyFilter(
          {pt_m4, pt_m3, pt_m2, pt_m1, pt, pt_p1, pt_p2, pt_p3, pt_p4});
        pt_m4 = pt_m3; pt_m3 = pt_m2; pt_m2 = pt_m1; pt_m1 = pt;
        pt = pt_p1; pt_p1 = pt_p2; pt_p2 = pt_p3; pt_p3 = pt_p4;
        pt_p4 = (idx + 5 < num_sequences) ?
          initial_sequence(idx + 5) : initial_sequence(num_sequences);
      }
    };

  const models::ControlSequence initial_control_sequence = control_sequence;
  applyFilterOverAxis(
    control_sequence.vx, initial_control_sequence.vx,
    control_history[0].vx, control_history[1].vx,
    control_history[2].vx, control_history[3].vx);
  applyFilterOverAxis(
    control_sequence.wz, initial_control_sequence.wz,
    control_history[0].wz, control_history[1].wz,
    control_history[2].wz, control_history[3].wz);

  unsigned int offset = settings.shift_control_sequence ? 1 : 0;
  control_history[0] = control_history[1];
  control_history[1] = control_history[2];
  control_history[2] = control_history[3];
  control_history[3] = {control_sequence.vx(offset), control_sequence.wz(offset)};
}

inline unsigned int findClosestPathPt(
  const std::vector<float> & vec, const float dist, const unsigned int init = 0u)
{
  float distim1 = init != 0u ? vec[init] : 0.0f;
  float disti = 0.0f;
  const unsigned int size = vec.size();
  for (unsigned int i = init + 1; i != size; i++) {
    disti = vec[i];
    if (disti > dist) {
      if (i > 0 && dist - distim1 < disti - dist) {return i - 1;}
      return i;
    }
    distim1 = disti;
  }
  return size - 1;
}

struct Pose2D { float x, y, theta; };

/**
 * @brief Shift columns of an Eigen Array left (-1) or right (1) by one place.
 * Used in trajectory integration and control sequence shifting.
 */
inline void shiftColumnsByOnePlace(Eigen::Ref<Eigen::ArrayXXf> e, int direction)
{
  int size = e.size();
  if (size == 1) {return;}
  if (abs(direction) != 1) {
    throw std::logic_error("Invalid direction, only 1 and -1 are valid values.");
  }
  if ((e.cols() == 1 || e.rows() == 1) && size > 1) {
    auto start_ptr = direction == 1 ? e.data() + size - 2 : e.data() + 1;
    auto end_ptr = direction == 1 ? e.data() : e.data() + size - 1;
    while (start_ptr != end_ptr) {
      *(start_ptr + direction) = *start_ptr;
      start_ptr -= direction;
    }
    *(start_ptr + direction) = *start_ptr;
  } else {
    auto start_ptr = direction == 1 ?
      e.data() + size - 2 * e.rows() : e.data() + e.rows();
    auto end_ptr = direction == 1 ?
      e.data() : e.data() + size - e.rows();
    auto span = e.rows();
    while (start_ptr != end_ptr) {
      std::copy(start_ptr, start_ptr + span, start_ptr + direction * span);
      start_ptr -= (direction * span);
    }
    std::copy(start_ptr, start_ptr + span, start_ptr + direction * span);
  }
}

inline float clamp(const float lower_bound, const float upper_bound, const float input)
{
  return std::min(upper_bound, std::max(input, lower_bound));
}

}  // namespace mppi::utils

#endif  // ACKERMANN_MPPI__TOOLS__UTILS_HPP_
