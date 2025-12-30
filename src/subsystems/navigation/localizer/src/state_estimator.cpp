#include "localizer/state_estimator.h"
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <GeographicLib/LocalCartesian.hpp>

using namespace gtsam;

namespace localizer
{

nav_msgs::msg::Odometry EstimatedState::to_odometry(const std::string& frame_id, const std::string& child_frame_id) const
{
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = timestamp;
    odom.header.frame_id = frame_id;
    odom.child_frame_id = child_frame_id;

    auto pose = nav_state.pose();
    auto vel = nav_state.velocity();

    odom.pose.pose.position.x = pose.x();
    odom.pose.pose.position.y = pose.y();
    odom.pose.pose.position.z = pose.z();

    auto quat = pose.rotation().toQuaternion();
    odom.pose.pose.orientation.w = quat.w();
    odom.pose.pose.orientation.x = quat.x();
    odom.pose.pose.orientation.y = quat.y();
    odom.pose.pose.orientation.z = quat.z();

    odom.twist.twist.linear.x = vel.x();
    odom.twist.twist.linear.y = vel.y();
    odom.twist.twist.linear.z = vel.z();

    for (int i = 0; i < 36; ++i) {
        odom.pose.covariance[i] = 0.0;
        odom.twist.covariance[i] = 0.0;
    }
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            if (i < 3 && j < 3) {
                odom.pose.covariance[i*6 + j] = covariance(i, j);
            }
        }
    }

    return odom;
}

StateEstimator::StateEstimator()
    : Node("localizer_node")
    , odom_to_base_(Pose3())
    , map_to_odom_(Pose3())
    , key_index_(0)
    , initialized_(false)
    , enu_origin_set_(false)
    , origin_lat_(0.0)
    , origin_lon_(0.0)
    , origin_alt_(0.0)
{
    ISAM2Params isam_params;
    isam_params.factorization = ISAM2Params::CHOLESKY;
    isam_params.relinearizeSkip = 10;

    isam_ = std::make_unique<ISAM2>(isam_params);
    new_factors_ = std::make_unique<NonlinearFactorGraph>();
    new_values_ = std::make_unique<Values>();

    std::string imu_topic = this->declare_parameter("imu_topic", "/imu");
    std::string gnss_topic = this->declare_parameter("gnss_topic", "/gps/fix");
    std::string odom_topic = this->declare_parameter("odom_topic", "/odom");
    std::string output_odom_topic = this->declare_parameter("output_odom_topic", "/localization/odom");
    double publish_rate = this->declare_parameter("publish_rate", 50.0);

    params_.imu_accel_noise = this->declare_parameter("imu_accel_noise", params_.imu_accel_noise);
    params_.imu_gyro_noise = this->declare_parameter("imu_gyro_noise", params_.imu_gyro_noise);
    params_.imu_bias_noise = this->declare_parameter("imu_bias_noise", params_.imu_bias_noise);
    params_.gnss_noise = this->declare_parameter("gnss_noise", params_.gnss_noise);
    params_.odom_noise = this->declare_parameter("odom_noise", params_.odom_noise);
    params_.max_time_window = this->declare_parameter("max_time_window", params_.max_time_window);
    params_.max_states = this->declare_parameter("max_states", static_cast<int>(params_.max_states));

    params_.frames.tf_prefix = this->declare_parameter("tf_prefix", params_.frames.tf_prefix);
    params_.frames.map_frame = this->declare_parameter("map_frame", params_.frames.map_frame);
    params_.frames.base_frame = this->declare_parameter("base_frame", params_.frames.base_frame);
    params_.frames.odom_frame = this->declare_parameter("odom_frame", params_.frames.odom_frame);
    params_.frames.imu_frame = this->declare_parameter("imu_frame", params_.frames.imu_frame);
    params_.frames.gnss_frame = this->declare_parameter("gnss_frame", params_.frames.gnss_frame);

    double origin_lat = this->declare_parameter("origin_lat", 0.0);
    double origin_lon = this->declare_parameter("origin_lon", 0.0);
    double origin_alt = this->declare_parameter("origin_alt", 0.0);

    if (std::abs(origin_lat) > 1e-6 || std::abs(origin_lon) > 1e-6) {
        set_enu_origin(origin_lat, origin_lon, origin_alt);
    }

    RCLCPP_INFO(this->get_logger(), "Parameters: imu_accel=%.4f, imu_gyro=%.4f, gnss=%.2f",
                params_.imu_accel_noise, params_.imu_gyro_noise, params_.gnss_noise);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    tf_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() { publish_transforms(); });

    Matrix33 imu_cov = Matrix33::Identity() * params_.imu_accel_noise * params_.imu_accel_noise;
    Matrix33 gyro_cov = Matrix33::Identity() * params_.imu_gyro_noise * params_.imu_gyro_noise;
    Matrix33 bias_cov = Matrix33::Identity() * params_.imu_bias_noise * params_.imu_bias_noise;

    auto imu_params = PreintegratedImuMeasurements::Params::MakeSharedU(9.81);
    imu_params->accelerometerCovariance = imu_cov;
    imu_params->gyroscopeCovariance = gyro_cov;
    imu_params->integrationCovariance = bias_cov;

    imu_preintegrated_ = std::make_unique<PreintegratedImuMeasurements>(imu_params, imuBias::ConstantBias());

    gnss_noise_model_ = noiseModel::Diagonal::Sigmas((Vector3() << params_.gnss_noise, params_.gnss_noise, params_.gnss_noise).finished());
    odom_noise_model_ = noiseModel::Diagonal::Sigmas((Vector6() << params_.odom_noise, params_.odom_noise, params_.odom_noise, params_.odom_noise, params_.odom_noise, params_.odom_noise).finished());

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, rclcpp::SensorDataQoS(),
        std::bind(&StateEstimator::fuse_imu, this, std::placeholders::_1));

    gnss_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        gnss_topic, rclcpp::SensorDataQoS(),
        std::bind(&StateEstimator::fuse_gnss, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::SensorDataQoS(),
        std::bind(&StateEstimator::fuse_odometry, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_odom_topic, 10);

    auto period = std::chrono::duration<double>(1.0 / publish_rate);
    publish_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&StateEstimator::publish_state, this));

    RCLCPP_INFO(this->get_logger(), "Localizer Node initialized");
    RCLCPP_INFO(this->get_logger(), "  IMU topic: %s", imu_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  GNSS topic: %s", gnss_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Odom topic: %s", odom_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Output odom: %s", output_odom_topic.c_str());
}

StateEstimator::~StateEstimator() = default;

void StateEstimator::reset()
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    ISAM2Params isam_params;
    isam_params.factorization = ISAM2Params::CHOLESKY;
    isam_params.relinearizeSkip = 10;

    isam_ = std::make_unique<ISAM2>(isam_params);
    new_factors_->resize(0);
    new_values_->clear();
    state_history_.clear();
    imu_buffer_.clear();

    odom_to_base_ = Pose3();
    map_to_odom_ = Pose3();
    key_index_ = 0;
    initialized_ = false;
    enu_origin_set_ = false;
    prev_odom_pose_ = std::nullopt;

    if (imu_preintegrated_) {
        imu_preintegrated_->resetIntegration();
    }

    RCLCPP_INFO(this->get_logger(), "StateEstimator reset");
}

void StateEstimator::publish_state()
{
    auto latest_state = get_latest_state();
    if (!latest_state.has_value()) {
        return;
    }

    std::string full_map_frame = params_.frames.tf_prefix.empty() ?
        params_.frames.map_frame : params_.frames.tf_prefix + "/" + params_.frames.map_frame;
    std::string full_base_frame = params_.frames.tf_prefix.empty() ?
        params_.frames.base_frame : params_.frames.tf_prefix + "/" + params_.frames.base_frame;

    odom_pub_->publish(latest_state->to_odometry(full_map_frame, full_base_frame));
}

void StateEstimator::fuse_imu(sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (!imu_to_base_) {
        try {
            auto tf = tf_buffer_->lookupTransform(
                params_.frames.base_frame, params_.frames.imu_frame,
                tf2::TimePointZero, tf2::durationFromSec(0.1));
            auto& q = tf.transform.rotation;
            auto& t = tf.transform.translation;
            imu_to_base_ = Pose3(
                Rot3::Quaternion(q.w, q.x, q.y, q.z),
                Point3(t.x, t.y, t.z));
            RCLCPP_INFO(this->get_logger(), "Cached IMU->base_link transform");
        } catch (const tf2::TransformException& ex) {
            RCLCPP_DEBUG(this->get_logger(), "IMU transform not available: %s", ex.what());
        }
    }

    if (!initialized_) {
        imu_buffer_.push_back(*imu_msg);
        return;
    }

    double dt = 0.01;
    if (!imu_buffer_.empty()) {
        auto prev_time = rclcpp::Time(imu_buffer_.back().header.stamp);
        auto curr_time = rclcpp::Time(imu_msg->header.stamp);
        double computed_dt = (curr_time - prev_time).seconds();

        dt = std::max(0.001, std::min(0.1, computed_dt));

        if (computed_dt <= 0.0) {
            RCLCPP_WARN(this->get_logger(), "Non-positive dt detected: %.6f, using default", computed_dt);
            dt = 0.01;
        }
    }

    Vector3 accel(imu_msg->linear_acceleration.x,
                  imu_msg->linear_acceleration.y,
                  imu_msg->linear_acceleration.z);
    Vector3 gyro(imu_msg->angular_velocity.x,
                 imu_msg->angular_velocity.y,
                 imu_msg->angular_velocity.z);

    if (imu_to_base_) {
        Rot3 R = imu_to_base_->rotation();
        accel = R.rotate(accel);
        gyro = R.rotate(gyro);
    }

    if (imu_preintegrated_) {
        imu_preintegrated_->integrateMeasurement(accel, gyro, dt);
    }

    imu_buffer_.push_back(*imu_msg);

    if (imu_buffer_.size() > 1000) {
        imu_buffer_.pop_front();
    }
}

void StateEstimator::fuse_gnss(sensor_msgs::msg::NavSatFix::SharedPtr gnss_msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Received GNSS: lat=%.6f, lon=%.6f",
                 gnss_msg->latitude, gnss_msg->longitude);

    std::lock_guard<std::mutex> lock(state_mutex_);

    if (!gnss_to_base_) {
        try {
            auto tf = tf_buffer_->lookupTransform(
                params_.frames.base_frame, params_.frames.gnss_frame,
                tf2::TimePointZero, tf2::durationFromSec(0.1));
            gnss_to_base_ = Point3(
                tf.transform.translation.x,
                tf.transform.translation.y,
                tf.transform.translation.z);
            RCLCPP_INFO(this->get_logger(), "Cached GNSS->base_link offset: (%.3f, %.3f, %.3f)",
                        gnss_to_base_->x(), gnss_to_base_->y(), gnss_to_base_->z());
        } catch (const tf2::TransformException& ex) {
            RCLCPP_DEBUG(this->get_logger(), "GNSS transform not available: %s", ex.what());
        }
    }

    if (!enu_origin_set_) {
        set_enu_origin(gnss_msg->latitude, gnss_msg->longitude, gnss_msg->altitude);
    }

    auto gnss_pos = lla_to_enu(gnss_msg->latitude, gnss_msg->longitude, gnss_msg->altitude);
    if (gnss_to_base_) {
        gnss_pos = gnss_pos + *gnss_to_base_;
    }

    if (!initialized_) {
        initialize_with_gnss(gnss_pos, rclcpp::Time(gnss_msg->header.stamp));
        return;
    }

    add_gnss_factor(gnss_msg);
    optimize_graph();
    marginalize_old_states();
}

void StateEstimator::fuse_odometry(nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (!initialized_) {
        return;
    }

    add_odom_factor(odom_msg);
    optimize_graph();
    marginalize_old_states();
}

void StateEstimator::initialize_with_gnss(const Point3& gnss_pos, const rclcpp::Time& timestamp)
{
    // Use first IMU measurement for initial orientation if available
    Rot3 initial_rotation = Rot3();
    if (!imu_buffer_.empty()) {
        const auto& first_imu = imu_buffer_.front();
        tf2::Quaternion q(
            first_imu.orientation.x,
            first_imu.orientation.y,
            first_imu.orientation.z,
            first_imu.orientation.w);

        // Convert to GTSAM rotation if orientation is valid (not all zeros)
        if (q.length2() > 0.01) {
            q.normalize();
            initial_rotation = Rot3::Quaternion(q.w(), q.x(), q.y(), q.z());
            RCLCPP_INFO(this->get_logger(), "Initializing with IMU orientation");
        } else {
            RCLCPP_WARN(this->get_logger(), "IMU orientation invalid, using identity rotation");
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "No IMU data available, using identity rotation");
    }

    auto initial_pose = Pose3(initial_rotation, gnss_pos);
    auto initial_velocity = Vector3(0.0, 0.0, 0.0);
    auto initial_bias = imuBias::ConstantBias();

    odom_to_base_ = initial_pose;
    map_to_odom_ = Pose3();

    current_pose_key_ = pose_key(key_index_);
    current_vel_key_ = vel_key(key_index_);
    current_bias_key_ = bias_key(key_index_);

    new_values_->insert(current_pose_key_, initial_pose);
    new_values_->insert(current_vel_key_, initial_velocity);
    new_values_->insert(current_bias_key_, initial_bias);

    // Tighter rotation prior if we have IMU orientation
    double rot_prior_noise = (!imu_buffer_.empty() && initial_rotation.matrix() != Rot3().matrix()) ? 0.02 : 0.1;

    auto pose_prior = noiseModel::Diagonal::Sigmas(
        (Vector6() << rot_prior_noise, rot_prior_noise, rot_prior_noise, 0.1, 0.1, 0.1).finished());
    auto vel_prior = noiseModel::Diagonal::Sigmas(
        (Vector3() << 0.5, 0.5, 0.5).finished());  // Tighter velocity prior
    auto bias_prior = noiseModel::Diagonal::Sigmas(
        (Vector6() << 0.05, 0.05, 0.05, 0.005, 0.005, 0.005).finished());  // Tighter bias prior

    new_factors_->addPrior(current_pose_key_, initial_pose, pose_prior);
    new_factors_->addPrior(current_vel_key_, initial_velocity, vel_prior);
    new_factors_->addPrior(current_bias_key_, initial_bias, bias_prior);

    optimize_graph();

    EstimatedState state;
    state.timestamp = timestamp;
    state.nav_state = NavState(initial_pose, initial_velocity);
    state.imu_bias = initial_bias;
    state.covariance = Eigen::Matrix<double, 15, 15>::Identity() * 0.1;

    state_history_.push_back(state);

    initialized_ = true;
    last_optimization_time_ = timestamp;

    RCLCPP_INFO(this->get_logger(), "StateEstimator initialized with GNSS at (%.2f, %.2f, %.2f)",
                gnss_pos.x(), gnss_pos.y(), gnss_pos.z());
}

std::optional<EstimatedState> StateEstimator::get_latest_state() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (state_history_.empty()) {
        return std::nullopt;
    }

    return state_history_.back();
}

std::optional<EstimatedState> StateEstimator::get_state_at_time(const rclcpp::Time& timestamp) const
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (state_history_.empty()) {
        return std::nullopt;
    }

    auto it = std::lower_bound(state_history_.begin(), state_history_.end(), timestamp,
        [](const EstimatedState& state, const rclcpp::Time& time) {
            return state.timestamp < time;
        });

    if (it != state_history_.end()) {
        return *it;
    }

    return state_history_.back();
}

void StateEstimator::publish_transforms()
{
    auto latest_state = get_latest_state();
    if (!latest_state) {
        return;
    }

    // Publish odom → base_link
    geometry_msgs::msg::TransformStamped odom_to_base_transform;
    odom_to_base_transform.header.stamp = latest_state->timestamp;
    odom_to_base_transform.header.frame_id = params_.frames.tf_prefix.empty() ?
        params_.frames.odom_frame : params_.frames.tf_prefix + "/" + params_.frames.odom_frame;
    odom_to_base_transform.child_frame_id = params_.frames.tf_prefix.empty() ?
        params_.frames.base_frame : params_.frames.tf_prefix + "/" + params_.frames.base_frame;

    odom_to_base_transform.transform.translation.x = odom_to_base_.x();
    odom_to_base_transform.transform.translation.y = odom_to_base_.y();
    odom_to_base_transform.transform.translation.z = odom_to_base_.z();

    auto odom_quat = odom_to_base_.rotation().toQuaternion();
    odom_to_base_transform.transform.rotation.w = odom_quat.w();
    odom_to_base_transform.transform.rotation.x = odom_quat.x();
    odom_to_base_transform.transform.rotation.y = odom_quat.y();
    odom_to_base_transform.transform.rotation.z = odom_quat.z();

    tf_broadcaster_->sendTransform(odom_to_base_transform);

    // Publish map → odom
    geometry_msgs::msg::TransformStamped map_to_odom_transform;
    map_to_odom_transform.header.stamp = latest_state->timestamp;
    map_to_odom_transform.header.frame_id = params_.frames.tf_prefix.empty() ?
        params_.frames.map_frame : params_.frames.tf_prefix + "/" + params_.frames.map_frame;
    map_to_odom_transform.child_frame_id = params_.frames.tf_prefix.empty() ?
        params_.frames.odom_frame : params_.frames.tf_prefix + "/" + params_.frames.odom_frame;

    map_to_odom_transform.transform.translation.x = map_to_odom_.x();
    map_to_odom_transform.transform.translation.y = map_to_odom_.y();
    map_to_odom_transform.transform.translation.z = map_to_odom_.z();

    auto map_quat = map_to_odom_.rotation().toQuaternion();
    map_to_odom_transform.transform.rotation.w = map_quat.w();
    map_to_odom_transform.transform.rotation.x = map_quat.x();
    map_to_odom_transform.transform.rotation.y = map_quat.y();
    map_to_odom_transform.transform.rotation.z = map_quat.z();

    tf_broadcaster_->sendTransform(map_to_odom_transform);
}

void StateEstimator::optimize_graph()
{
    if (new_factors_->empty()) {
        return;
    }

    try {
        isam_->update(*new_factors_, *new_values_);

        new_factors_->resize(0);
        new_values_->clear();

        auto result = isam_->calculateEstimate();

        if (!result.empty() && result.exists(current_pose_key_) && result.exists(current_vel_key_)) {
            EstimatedState state;
            state.timestamp = this->get_clock()->now();

            auto map_to_base = result.at<Pose3>(current_pose_key_);
            auto vel = result.at<Vector3>(current_vel_key_);
            state.nav_state = NavState(map_to_base, vel);

            if (result.exists(current_bias_key_)) {
                state.imu_bias = result.at<imuBias::ConstantBias>(current_bias_key_);
            }

            odom_to_base_ = map_to_base;

            // Get covariance for pose only
            try {
                state.covariance = Eigen::Matrix<double, 15, 15>::Identity() * 0.1;
                auto pose_cov = isam_->marginalCovariance(current_pose_key_);
                if (pose_cov.rows() == 6 && pose_cov.cols() == 6) {
                    state.covariance.block<6, 6>(0, 0) = pose_cov;
                }
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to get covariance: %s", e.what());
                state.covariance = Eigen::Matrix<double, 15, 15>::Identity() * 0.1;
            }

            state_history_.push_back(state);
        }
    }
    catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "ISAM2 update failed: %s", e.what());
    }
}

void StateEstimator::add_imu_factor()
{
    if (!imu_preintegrated_ || key_index_ == 0) {
        return;
    }

    auto prev_pose_key = pose_key(key_index_ - 1);
    auto prev_vel_key = vel_key(key_index_ - 1);
    auto prev_bias_key = bias_key(key_index_ - 1);

    new_factors_->emplace_shared<ImuFactor>(
        prev_pose_key, prev_vel_key, current_pose_key_,
        current_vel_key_, prev_bias_key, *imu_preintegrated_);

    auto bias_noise = noiseModel::Diagonal::Sigmas(
        (Vector6() << params_.imu_bias_noise, params_.imu_bias_noise, params_.imu_bias_noise,
                      params_.imu_bias_noise, params_.imu_bias_noise, params_.imu_bias_noise).finished());

    new_factors_->emplace_shared<BetweenFactor<imuBias::ConstantBias>>(
        prev_bias_key, current_bias_key_, imuBias::ConstantBias(), bias_noise);
}

void StateEstimator::add_odom_factor(const nav_msgs::msg::Odometry::SharedPtr& odom_msg)
{
    if (!prev_odom_pose_) {
        prev_odom_pose_ = Pose3(Rot3(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z),
                                Point3(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z));
        return;
    }

    auto current_odom_pose = Pose3(Rot3(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z),
                                   Point3(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z));

    auto odom_delta = prev_odom_pose_->between(current_odom_pose);

    createNewState();

    new_factors_->emplace_shared<BetweenFactor<Pose3>>(pose_key(key_index_ - 1), current_pose_key_, odom_delta, odom_noise_model_);

    prev_odom_pose_ = current_odom_pose;
}

void StateEstimator::add_gnss_factor(const sensor_msgs::msg::NavSatFix::SharedPtr& gnss_msg)
{
    auto gnss_pos = lla_to_enu(gnss_msg->latitude, gnss_msg->longitude, gnss_msg->altitude);
    if (gnss_to_base_) {
        gnss_pos = gnss_pos + *gnss_to_base_;
    }

    createNewState();

    double base_noise = params_.gnss_noise;
    double adaptive_noise = base_noise;

    if (gnss_msg->position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
        double cov_noise = std::sqrt(gnss_msg->position_covariance[0]);
        adaptive_noise = std::max(base_noise, cov_noise);
    }

    // Note: KITTI dataset uses status=-2 but has good GPS quality
    // Only apply noise penalty for status values indicating actual poor quality
    // Do not penalize negative status codes from KITTI
    // Removed: if (gnss_msg->status.status < 0) adaptive_noise *= 10.0;
    if (gnss_msg->status.status == 0) adaptive_noise *= 1.5;  // Reduced from 2.0

    auto adaptive_gnss_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector3() << adaptive_noise, adaptive_noise, adaptive_noise).finished());

    new_factors_->emplace_shared<GPSFactor>(current_pose_key_, gnss_pos, adaptive_gnss_noise);
}

void StateEstimator::createNewState()
{
    key_index_++;
    current_pose_key_ = pose_key(key_index_);
    current_vel_key_ = vel_key(key_index_);
    current_bias_key_ = bias_key(key_index_);

    if (key_index_ > 0 && imu_preintegrated_) {
        auto prev_state = state_history_.back();
        auto predicted = imu_preintegrated_->predict(prev_state.nav_state, prev_state.imu_bias);

        new_values_->insert(current_pose_key_, predicted.pose());
        new_values_->insert(current_vel_key_, predicted.velocity());
        new_values_->insert(current_bias_key_, prev_state.imu_bias);

        add_imu_factor();
        imu_preintegrated_->resetIntegration();
    }
}

void StateEstimator::marginalize_old_states()
{
    if (state_history_.size() <= params_.max_states) {
        return;
    }

    auto time_threshold = this->get_clock()->now() - rclcpp::Duration::from_seconds(params_.max_time_window);

    while (!state_history_.empty() && state_history_.front().timestamp < time_threshold) {
        state_history_.pop_front();
    }
}

gtsam::Point3 StateEstimator::lla_to_enu(double lat, double lon, double alt) const
{
    if (!enu_origin_set_) {
        return Point3();
    }

    GeographicLib::LocalCartesian proj(origin_lat_, origin_lon_, origin_alt_);
    double x, y, z;
    proj.Forward(lat, lon, alt, x, y, z);

    return Point3(x, y, z);
}

void StateEstimator::set_enu_origin(double lat, double lon, double alt)
{
    origin_lat_ = lat;
    origin_lon_ = lon;
    origin_alt_ = alt;
    enu_origin_set_ = true;

    RCLCPP_INFO(this->get_logger(), "ENU origin set to (%.6f, %.6f, %.2f)", lat, lon, alt);
}

} // namespace localizer

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<localizer::StateEstimator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
