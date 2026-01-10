#include "localizer/state_estimator.h"

#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <cinttypes>

namespace localizer
{

    // ============================================================================
    // EstimatedState Implementation
    // ============================================================================

    nav_msgs::msg::Odometry EstimatedState::to_odometry(
        const std::string &frame_id,
        const std::string &child_frame_id) const
    {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = timestamp;
        odom.header.frame_id = frame_id;
        odom.child_frame_id = child_frame_id;

        const auto &pose = nav_state.pose();
        const auto &vel = nav_state.velocity();

        odom.pose.pose.position.x = pose.x();
        odom.pose.pose.position.y = pose.y();
        odom.pose.pose.position.z = pose.z();

        auto quat = pose.rotation().toQuaternion();
        odom.pose.pose.orientation.w = quat.w();
        odom.pose.pose.orientation.x = quat.x();
        odom.pose.pose.orientation.y = quat.y();
        odom.pose.pose.orientation.z = quat.z();

        // Velocity is already in the correct frame from nav_state
        odom.twist.twist.linear.x = vel.x();
        odom.twist.twist.linear.y = vel.y();
        odom.twist.twist.linear.z = vel.z();

        // Fill pose covariance (6x6 block from 15x15 state covariance)
        // ROS uses [x, y, z, roll, pitch, yaw], GTSAM uses [roll, pitch, yaw, x, y, z]
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                odom.pose.covariance[i * 6 + j] = covariance(3 + i, 3 + j);
                odom.pose.covariance[(3 + i) * 6 + (3 + j)] = covariance(i, j);
            }
        }

        // Fill twist covariance
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                odom.twist.covariance[i * 6 + j] = covariance(6 + i, 6 + j);
            }
        }

        return odom;
    }

    // ============================================================================
    // StateEstimator Constructor & Destructor
    // ============================================================================

    StateEstimator::StateEstimator()
        : Node("state_estimator"), current_key_index_(0), oldest_key_index_(0), enu_origin_set_(false), origin_lat_(0.0), origin_lon_(0.0), origin_alt_(0.0), filter_initialized_(false)
    {
        init_state_.store(InitState::WAITING_FOR_IMU);

        declare_parameters();
        validate_parameters();

        // Initialize ISAM2
        gtsam::ISAM2Params isam_params;
        isam_params.relinearizeThreshold = params_.isam2_relinearize_threshold;
        isam_params.relinearizeSkip = params_.isam2_relinearize_skip;
        isam_params.findUnusedFactorSlots = true;
        isam_ = std::make_unique<gtsam::ISAM2>(isam_params);

        // Initialize IMU preintegration parameters
        init_imu_params();

        // Pre-compute noise models
        init_noise_models();

        // Cache frame IDs
        cache_frame_ids();

        // Initialize timing
        min_state_dt_ = 1.0 / params_.state_creation_rate;
        last_state_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

        // Setup TF
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Setup subscribers
        std::string imu_topic = this->get_parameter("imu_topic").as_string();
        std::string gnss_topic = this->get_parameter("gnss_topic").as_string();
        std::string odom_topic = this->get_parameter("odom_topic").as_string();

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, rclcpp::SensorDataQoS(),
            std::bind(&StateEstimator::imu_callback, this, std::placeholders::_1));

        gnss_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            gnss_topic, rclcpp::SensorDataQoS(),
            std::bind(&StateEstimator::gnss_callback, this, std::placeholders::_1));

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, rclcpp::SensorDataQoS(),
            std::bind(&StateEstimator::odom_callback, this, std::placeholders::_1));

        // Setup publishers
        std::string output_topic = this->get_parameter("output_odom_topic").as_string();
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic, 10);
        filtered_imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/filtered", rclcpp::SensorDataQoS());

        // Setup timers
        double publish_rate = this->get_parameter("publish_rate").as_double();
        publish_timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_rate),
            std::bind(&StateEstimator::publish_state, this));

        tf_timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / params_.tf_publish_rate),
            std::bind(&StateEstimator::publish_transforms, this));

        RCLCPP_INFO(get_logger(), "StateEstimator initialized");
        RCLCPP_INFO(get_logger(), "  State: WAITING_FOR_IMU");
        RCLCPP_INFO(get_logger(), "  IMU topic: %s",
                    this->get_parameter("imu_topic").as_string().c_str());
        RCLCPP_INFO(get_logger(), "  GNSS topic: %s",
                    this->get_parameter("gnss_topic").as_string().c_str());
        RCLCPP_INFO(get_logger(), "  Odom topic: %s",
                    this->get_parameter("odom_topic").as_string().c_str());
    }

    StateEstimator::~StateEstimator() = default;

    void StateEstimator::declare_parameters()
    {
        // Topics
        declare_parameter("imu_topic", "/imu");
        declare_parameter("gnss_topic", "/gps/fix");
        declare_parameter("odom_topic", "/odom/ground_truth");
        declare_parameter("output_odom_topic", "/localization/odom");
        declare_parameter("publish_rate", 50.0);

        // IMU noise
        params_.accel_noise_sigma = declare_parameter("accel_noise_sigma", params_.accel_noise_sigma);
        params_.gyro_noise_sigma = declare_parameter("gyro_noise_sigma", params_.gyro_noise_sigma);
        params_.accel_bias_rw_sigma = declare_parameter("accel_bias_rw_sigma", params_.accel_bias_rw_sigma);
        params_.gyro_bias_rw_sigma = declare_parameter("gyro_bias_rw_sigma", params_.gyro_bias_rw_sigma);

        // Sensor noise
        params_.gnss_position_sigma = declare_parameter("gnss_position_sigma", params_.gnss_position_sigma);
        params_.gnss_altitude_sigma = declare_parameter("gnss_altitude_sigma", 0.15);
        params_.odom_position_sigma = declare_parameter("odom_position_sigma", params_.odom_position_sigma);
        params_.odom_rotation_sigma = declare_parameter("odom_rotation_sigma", params_.odom_rotation_sigma);

        // Graph management
        params_.min_imu_samples_for_init = declare_parameter("min_imu_samples_for_init",
                                                             static_cast<int>(params_.min_imu_samples_for_init));
        params_.state_creation_rate = declare_parameter("state_creation_rate", params_.state_creation_rate);
        params_.max_graph_states = declare_parameter("max_graph_states",
                                                     static_cast<int>(params_.max_graph_states));
        params_.gnss_max_age = declare_parameter("gnss_max_age", params_.gnss_max_age);

        // Frames
        params_.frames.map_frame = declare_parameter("map_frame", params_.frames.map_frame);
        params_.frames.odom_frame = declare_parameter("odom_frame", params_.frames.odom_frame);
        params_.frames.base_frame = declare_parameter("base_frame", params_.frames.base_frame);
        params_.frames.imu_frame = declare_parameter("imu_frame", params_.frames.imu_frame);
        params_.frames.gnss_frame = declare_parameter("gnss_frame", params_.frames.gnss_frame);
        params_.frames.tf_prefix = declare_parameter("tf_prefix", params_.frames.tf_prefix);

        // ISAM2 parameters
        params_.isam2_relinearize_threshold = declare_parameter("isam2_relinearize_threshold",
                                                                params_.isam2_relinearize_threshold);
        params_.isam2_relinearize_skip = declare_parameter("isam2_relinearize_skip",
                                                           params_.isam2_relinearize_skip);

        // IMU integration parameters
        params_.integration_covariance = declare_parameter("integration_covariance",
                                                           params_.integration_covariance);
        params_.bias_acc_omega_int = declare_parameter("bias_acc_omega_int",
                                                       params_.bias_acc_omega_int);

        // Initialization parameters
        params_.initial_velocity_sigma = declare_parameter("initial_velocity_sigma",
                                                           params_.initial_velocity_sigma);
        params_.gnss_fix_quality_multiplier = declare_parameter("gnss_fix_quality_multiplier",
                                                                params_.gnss_fix_quality_multiplier);

        // GNSS outlier rejection
        params_.use_robust_gnss_noise = declare_parameter("use_robust_gnss_noise",
                                                          params_.use_robust_gnss_noise);
        params_.gnss_huber_k = declare_parameter("gnss_huber_k", params_.gnss_huber_k);

        // Bias prior parameters
        params_.initial_accel_bias_sigma = declare_parameter("initial_accel_bias_sigma",
                                                             params_.initial_accel_bias_sigma);
        params_.initial_gyro_bias_sigma = declare_parameter("initial_gyro_bias_sigma",
                                                            params_.initial_gyro_bias_sigma);

        // Initial pose prior
        params_.initial_rotation_sigma = declare_parameter("initial_rotation_sigma",
                                                           params_.initial_rotation_sigma);

        // Buffer and timing
        params_.imu_buffer_max_size = declare_parameter("imu_buffer_max_size",
                                                        static_cast<int>(params_.imu_buffer_max_size));
        params_.tf_publish_rate = declare_parameter("tf_publish_rate", params_.tf_publish_rate);

        // IMU filtering (EMA)
        params_.enable_imu_filter = declare_parameter("enable_imu_filter", params_.enable_imu_filter);
        params_.imu_filter_alpha = declare_parameter("imu_filter_alpha", params_.imu_filter_alpha);

        // ENU origin (optional)
        double origin_lat = declare_parameter("origin_lat", 0.0);
        double origin_lon = declare_parameter("origin_lon", 0.0);
        double origin_alt = declare_parameter("origin_alt", 0.0);

        if (std::abs(origin_lat) > 1e-6 || std::abs(origin_lon) > 1e-6)
        {
            set_enu_origin(origin_lat, origin_lon, origin_alt);
        }
    }

    void StateEstimator::validate_parameters()
    {
        bool valid = true;

        auto check_positive = [this, &valid](double val, const char *name)
        {
            if (val <= 0.0)
            {
                RCLCPP_ERROR(get_logger(), "%s must be positive, got %.6f", name, val);
                valid = false;
            }
        };

        check_positive(params_.accel_noise_sigma, "accel_noise_sigma");
        check_positive(params_.gyro_noise_sigma, "gyro_noise_sigma");
        check_positive(params_.accel_bias_rw_sigma, "accel_bias_rw_sigma");
        check_positive(params_.gyro_bias_rw_sigma, "gyro_bias_rw_sigma");
        check_positive(params_.gnss_position_sigma, "gnss_position_sigma");
        check_positive(params_.odom_position_sigma, "odom_position_sigma");
        check_positive(params_.odom_rotation_sigma, "odom_rotation_sigma");
        check_positive(params_.state_creation_rate, "state_creation_rate");
        check_positive(params_.gnss_max_age, "gnss_max_age");
        check_positive(params_.gnss_huber_k, "gnss_huber_k");
        check_positive(params_.initial_accel_bias_sigma, "initial_accel_bias_sigma");
        check_positive(params_.initial_gyro_bias_sigma, "initial_gyro_bias_sigma");
        check_positive(params_.initial_rotation_sigma, "initial_rotation_sigma");
        check_positive(params_.tf_publish_rate, "tf_publish_rate");
        check_positive(params_.initial_velocity_sigma, "initial_velocity_sigma");

        if (!valid)
        {
            throw std::runtime_error("Invalid StateEstimator parameters");
        }
    }

    void StateEstimator::init_imu_params()
    {
        imu_params_ = boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params>(
            new gtsam::PreintegratedCombinedMeasurements::Params());

        // Gravity vector (verified correct for your setup)
        imu_params_->n_gravity = gtsam::Vector3(0.0, 0.0, -9.81);

        // DEBUG: Log gravity vector configuration
        RCLCPP_INFO(get_logger(), "DEBUG: Configured gravity vector: (%.3f, %.3f, %.3f)",
                    imu_params_->n_gravity.x(), imu_params_->n_gravity.y(), imu_params_->n_gravity.z());

        // Compute variance values
        double accel_xy_var = params_.accel_noise_sigma * params_.accel_noise_sigma;
        double gyro_var = params_.gyro_noise_sigma * params_.gyro_noise_sigma;
        double accel_bias_var = params_.accel_bias_rw_sigma * params_.accel_bias_rw_sigma;
        double gyro_bias_var = params_.gyro_bias_rw_sigma * params_.gyro_bias_rw_sigma;

        // ANISOTROPIC ACCELEROMETER NOISE
        // Z-axis (vertical) is less accurate due to:
        // 1. Gravity dominance (~9.81 m/s^2) makes small bias errors significant
        // 2. Poor observability of vertical velocity without barometer
        // 3. Rapid drift accumulation in vertical position
        // Solution: Increase Z-axis uncertainty by factor of 2-4
        double accel_z_var = accel_xy_var * 10.0; // 2x higher standard deviation

        gtsam::Matrix3 accel_cov = gtsam::Matrix3::Zero();
        accel_cov(0, 0) = accel_xy_var; // X (forward/back)
        accel_cov(1, 1) = accel_xy_var; // Y (left/right)
        accel_cov(2, 2) = accel_z_var;  // Z (up/down) - HIGHER uncertainty
        imu_params_->accelerometerCovariance = accel_cov;

        RCLCPP_INFO(get_logger(),
                    "Accelerometer covariance: XY_sigma=%.4f, Z_sigma=%.4f (%.1fx higher)",
                    params_.accel_noise_sigma,
                    std::sqrt(accel_z_var),
                    std::sqrt(accel_z_var) / params_.accel_noise_sigma);

        // ISOTROPIC GYROSCOPE NOISE
        // Gyroscope uncertainty is typically similar across all axes
        imu_params_->gyroscopeCovariance = gtsam::I_3x3 * gyro_var;

        // BIAS RANDOM WALK COVARIANCES
        // Could also make these anisotropic, but typically less critical
        imu_params_->biasAccCovariance = gtsam::I_3x3 * accel_bias_var;
        imu_params_->biasOmegaCovariance = gtsam::I_3x3 * gyro_bias_var;

        // Integration uncertainty (position uncertainty growth during preintegration)
        imu_params_->integrationCovariance = gtsam::I_3x3 * params_.integration_covariance;

        // Bias coupling term
        imu_params_->biasAccOmegaInt = gtsam::I_6x6 * params_.bias_acc_omega_int;

        RCLCPP_INFO(get_logger(),
                    "IMU params: accel_sigma_xy=%.4f, accel_sigma_z=%.4f, gyro_sigma=%.4f",
                    params_.accel_noise_sigma,
                    std::sqrt(accel_z_var),
                    params_.gyro_noise_sigma);

        RCLCPP_INFO(get_logger(),
                    "IMU bias random walk: accel_bias_sigma=%.6f, gyro_bias_sigma=%.6f",
                    params_.accel_bias_rw_sigma,
                    params_.gyro_bias_rw_sigma);
    }

    void StateEstimator::init_noise_models()
    {
        // Initial pose prior noise
        initial_pose_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector6() << params_.initial_rotation_sigma,
             params_.initial_rotation_sigma,
             params_.initial_rotation_sigma,
             params_.gnss_position_sigma,
             params_.gnss_position_sigma,
             params_.gnss_position_sigma)
                .finished());

        // Initial velocity prior noise
        initial_vel_noise_ = gtsam::noiseModel::Isotropic::Sigma(3, params_.initial_velocity_sigma);

        // Initial bias prior noise
        initial_bias_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector6() << params_.initial_accel_bias_sigma,
             params_.initial_accel_bias_sigma,
             params_.initial_accel_bias_sigma,
             params_.initial_gyro_bias_sigma,
             params_.initial_gyro_bias_sigma,
             params_.initial_gyro_bias_sigma)
                .finished());

        // Odometry noise (typically horizontal is more accurate)
        odom_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector6() << params_.odom_rotation_sigma,
             params_.odom_rotation_sigma,
             params_.odom_rotation_sigma,
             params_.odom_position_sigma,
             params_.odom_position_sigma,
             params_.odom_position_sigma)
                .finished()); // Slightly less trust in Z

        RCLCPP_INFO(get_logger(),
                    "Noise models initialized: pose_sigma=%.3f, vel_sigma=%.3f, odom_sigma=%.3f",
                    params_.gnss_position_sigma,
                    params_.initial_velocity_sigma,
                    params_.odom_position_sigma);
    }
    void StateEstimator::cache_frame_ids()
    {
        auto add_prefix = [this](const std::string &frame)
        {
            if (params_.frames.tf_prefix.empty())
            {
                return frame;
            }
            return params_.frames.tf_prefix + "/" + frame;
        };

        cached_frames_.map = add_prefix(params_.frames.map_frame);
        cached_frames_.odom = add_prefix(params_.frames.odom_frame);
        cached_frames_.base = add_prefix(params_.frames.base_frame);
    }

    // ============================================================================
    // Reset
    // ============================================================================

    void StateEstimator::reset()
    {
        // Lock in consistent order: state first, then imu
        std::scoped_lock lock(state_mutex_, imu_mutex_);

        init_state_.store(InitState::WAITING_FOR_IMU);

        // Reset ISAM2
        gtsam::ISAM2Params isam_params;
        isam_params.relinearizeThreshold = params_.isam2_relinearize_threshold;
        isam_params.relinearizeSkip = params_.isam2_relinearize_skip;
        isam_params.findUnusedFactorSlots = true;
        isam_ = std::make_unique<gtsam::ISAM2>(isam_params);

        current_key_index_ = 0;
        oldest_key_index_ = 0;
        graph_states_.clear();
        imu_buffer_.clear();
        preintegrated_imu_.reset();

        pending_gnss_.reset();
        pending_odom_.reset();

        last_state_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        imu_to_base_.reset();
        gnss_to_base_.reset();

        // Reset EMA filter state
        filtered_accel_.reset();
        filtered_gyro_.reset();
        filter_initialized_ = false;

        RCLCPP_INFO(get_logger(), "StateEstimator reset to WAITING_FOR_IMU");
    }

    InitState StateEstimator::get_init_state() const
    {
        return init_state_.load();
    }

    // ============================================================================
    // Sensor Callbacks
    // ============================================================================

    void StateEstimator::imu_callback(sensor_msgs::msg::Imu::SharedPtr msg)
    {
        lookup_sensor_transforms();

        // =========================================================================
        // BUG FIX: Original code had z-component hardcoded to 0!
        // Original: Eigen::Vector3d accel(msg->linear_acceleration.x, msg->linear_acceleration.y, 0);
        // =========================================================================
        Eigen::Vector3d accel_raw(
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z); // FIXED: was hardcoded to 0!
        Eigen::Vector3d gyro_raw(
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z);

        Eigen::Vector3d accel = accel_raw;
        Eigen::Vector3d gyro = gyro_raw;

        // DEBUG: Log raw IMU data periodically
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "DEBUG IMU raw: accel=(%.3f, %.3f, %.3f) |a|=%.3f, gyro=(%.3f, %.3f, %.3f)",
                             accel_raw.x(), accel_raw.y(), accel_raw.z(), accel_raw.norm(),
                             gyro_raw.x(), gyro_raw.y(), gyro_raw.z());

        // Transform to base_link frame if transform available
        if (imu_to_base_)
        {
            gtsam::Rot3 R = imu_to_base_->rotation();
            accel = R.rotate(gtsam::Point3(accel_raw));
            gyro = R.rotate(gtsam::Point3(gyro_raw));

            // DEBUG: Log the transformation effect
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "DEBUG IMU transform: raw_accel=(%.3f, %.3f, %.3f) -> base_accel=(%.3f, %.3f, %.3f)",
                                 accel_raw.x(), accel_raw.y(), accel_raw.z(),
                                 accel.x(), accel.y(), accel.z());

            // DEBUG: Log the rotation matrix (once)
            static bool logged_rotation = false;
            if (!logged_rotation)
            {
                Eigen::Matrix3d rot_matrix = R.matrix();
                RCLCPP_INFO(get_logger(),
                            "DEBUG IMU->base rotation matrix:\n"
                            "  [%.3f, %.3f, %.3f]\n"
                            "  [%.3f, %.3f, %.3f]\n"
                            "  [%.3f, %.3f, %.3f]",
                            rot_matrix(0, 0), rot_matrix(0, 1), rot_matrix(0, 2),
                            rot_matrix(1, 0), rot_matrix(1, 1), rot_matrix(1, 2),
                            rot_matrix(2, 0), rot_matrix(2, 1), rot_matrix(2, 2));
                logged_rotation = true;
            }
        }
        else
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "DEBUG: No imu_to_base transform available - using raw IMU data");
        }

        // Apply EMA filtering if enabled
        if (params_.enable_imu_filter)
        {
            if (!filter_initialized_)
            {
                // Initialize filter with first measurement
                filtered_accel_ = accel;
                filtered_gyro_ = gyro;
                filter_initialized_ = true;
            }
            else
            {
                // Apply EMA: filtered = alpha * current + (1 - alpha) * previous
                double alpha = params_.imu_filter_alpha;
                filtered_accel_ = alpha * accel + (1.0 - alpha) * (*filtered_accel_);
                filtered_gyro_ = alpha * gyro + (1.0 - alpha) * (*filtered_gyro_);
            }

            // Use filtered values
            accel = *filtered_accel_;
            gyro = *filtered_gyro_;

            // Publish filtered IMU data
            auto filtered_msg = std::make_unique<sensor_msgs::msg::Imu>();
            filtered_msg->header = msg->header;
            filtered_msg->header.frame_id = params_.frames.base_frame; // Filtered data is in base frame

            filtered_msg->linear_acceleration.x = accel.x();
            filtered_msg->linear_acceleration.y = accel.y();
            filtered_msg->linear_acceleration.z = accel.z();

            filtered_msg->angular_velocity.x = gyro.x();
            filtered_msg->angular_velocity.y = gyro.y();
            filtered_msg->angular_velocity.z = gyro.z();

            // Copy orientation if available
            filtered_msg->orientation = msg->orientation;

            filtered_imu_pub_->publish(std::move(filtered_msg));
        }

        rclcpp::Time stamp(msg->header.stamp);

        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            imu_buffer_.emplace_back(stamp, accel, gyro);

            while (imu_buffer_.size() > params_.imu_buffer_max_size)
            {
                imu_buffer_.pop_front();
            }
        }

        // State machine with proper synchronization
        InitState current_state = init_state_.load();

        if (current_state == InitState::WAITING_FOR_IMU)
        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            if (imu_buffer_.size() >= params_.min_imu_samples_for_init)
            {
                init_state_.store(InitState::WAITING_FOR_POSITION);
                RCLCPP_INFO(get_logger(), "State: WAITING_FOR_POSITION");
            }
        }
        else if (current_state == InitState::RUNNING)
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            // Re-check state after acquiring lock to avoid race with gnss_callback
            if (init_state_.load() == InitState::RUNNING)
            {
                double dt = (stamp - last_state_time_).seconds();
                if (dt >= min_state_dt_)
                {
                    create_new_state(stamp);
                }
            }
        }
    }

    void StateEstimator::gnss_callback(sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        lookup_sensor_transforms();

        if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX)
        {
            RCLCPP_DEBUG(get_logger(), "GNSS: No fix, ignoring");
            return;
        }

        rclcpp::Time stamp(msg->header.stamp);
        double age = (this->get_clock()->now() - stamp).seconds();
        if (age > params_.gnss_max_age)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "GNSS: Rejecting stale measurement (%.2fs old)", age);
            return;
        }

        if (!enu_origin_set_)
        {
            set_enu_origin(msg->latitude, msg->longitude, msg->altitude);
        }

        gtsam::Point3 enu_pos = lla_to_enu(msg->latitude, msg->longitude, msg->altitude);

        // DEBUG: Log GNSS position
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "DEBUG GNSS: LLA=(%.6f, %.6f, %.2f) -> ENU=(%.3f, %.3f, %.3f)",
                             msg->latitude, msg->longitude, msg->altitude,
                             enu_pos.x(), enu_pos.y(), enu_pos.z());

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "DEBUG GNSS position in meters: East=%.3f m, North=%.3f m, Up=%.3f m, distance_from_origin=%.3f m",
                             enu_pos.x(), enu_pos.y(), enu_pos.z(),
                             std::sqrt(enu_pos.x() * enu_pos.x() + enu_pos.y() * enu_pos.y() + enu_pos.z() * enu_pos.z()));

        // Determine noise from covariance if available
        double sigma = params_.gnss_position_sigma;
        if (msg->position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN)
        {
            double cov = 0.5 * (msg->position_covariance[0] + msg->position_covariance[4]);
            sigma = std::max(sigma, std::sqrt(cov));
        }

        if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_FIX)
        {
            sigma *= params_.gnss_fix_quality_multiplier;
        }

        InitState current_state = init_state_.load();

        if (current_state == InitState::WAITING_FOR_IMU)
        {
            RCLCPP_DEBUG(get_logger(), "GNSS: Waiting for IMU first");
        }
        else if (current_state == InitState::WAITING_FOR_POSITION)
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            // Re-check state after acquiring lock
            if (init_state_.load() == InitState::WAITING_FOR_POSITION)
            {
                if (initialize_graph(enu_pos, stamp))
                {
                    init_state_.store(InitState::RUNNING);
                    RCLCPP_INFO(get_logger(), "State: RUNNING");
                    RCLCPP_INFO(get_logger(), "Initialized at ENU (%.2f, %.2f, %.2f)",
                                enu_pos.x(), enu_pos.y(), enu_pos.z());
                }
            }
        }
        else if (current_state == InitState::RUNNING)
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            // Store raw measurement - offset applied when factor is created
            pending_gnss_.position = enu_pos;
            pending_gnss_.sigma = sigma;
            pending_gnss_.timestamp = stamp;
            pending_gnss_.valid = true;
        }
    }

    void StateEstimator::odom_callback(nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (init_state_.load() != InitState::RUNNING)
        {
            return;
        }

        gtsam::Pose3 current_pose(
            gtsam::Rot3::Quaternion(
                msg->pose.pose.orientation.w,
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z),
            gtsam::Point3(
                msg->pose.pose.position.x,
                msg->pose.pose.position.y,
                msg->pose.pose.position.z));

        std::lock_guard<std::mutex> lock(state_mutex_);

        if (!pending_odom_.prev_pose)
        {
            pending_odom_.prev_pose = current_pose;
            return;
        }

        gtsam::Pose3 delta = pending_odom_.prev_pose->between(current_pose);

        if (pending_odom_.valid)
        {
            pending_odom_.delta = pending_odom_.delta.compose(delta);
        }
        else
        {
            pending_odom_.delta = delta;
            pending_odom_.valid = true;
        }

        pending_odom_.prev_pose = current_pose;
    }

    // ============================================================================
    // Initialization
    // ============================================================================

    bool StateEstimator::initialize_graph(
        const gtsam::Point3 &position,
        const rclcpp::Time &timestamp)
    {
        // Estimate initial orientation from gravity (needs imu_mutex_)
        gtsam::Rot3 initial_rot;
        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            initial_rot = estimate_initial_orientation_locked();
        }

        // DEBUG: Log initial orientation
        auto rpy = initial_rot.rpy();
        RCLCPP_INFO(get_logger(),
                    "DEBUG Initial orientation: roll=%.3f deg, pitch=%.3f deg, yaw=%.3f deg",
                    rpy.x() * 180.0 / M_PI, rpy.y() * 180.0 / M_PI, rpy.z() * 180.0 / M_PI);

        gtsam::Pose3 initial_pose(initial_rot, position);
        gtsam::Vector3 initial_vel = gtsam::Vector3::Zero();
        gtsam::imuBias::ConstantBias initial_bias;

        gtsam::NonlinearFactorGraph factors;
        gtsam::Values values;

        values.insert(X(current_key_index_), initial_pose);
        values.insert(V(current_key_index_), initial_vel);
        values.insert(B(current_key_index_), initial_bias);

        factors.addPrior(X(current_key_index_), initial_pose, initial_pose_noise_);
        factors.addPrior(V(current_key_index_), initial_vel, initial_vel_noise_);
        factors.addPrior(B(current_key_index_), initial_bias, initial_bias_noise_);

        try
        {
            isam_->update(factors, values);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Initial ISAM2 update failed: %s", e.what());
            return false;
        }

        GraphStateEntry entry;
        entry.key_index = current_key_index_;
        entry.timestamp = timestamp;
        entry.nav_state = gtsam::NavState(initial_pose, initial_vel);
        entry.bias = initial_bias;
        graph_states_.push_back(entry);

        reset_preintegration(initial_bias);
        last_state_time_ = timestamp;

        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            prune_imu_buffer_locked(timestamp);
        }

        return true;
    }

    gtsam::Rot3 StateEstimator::estimate_initial_orientation_locked() const
    {
        if (imu_buffer_.empty())
        {
            RCLCPP_WARN(get_logger(), "No IMU data for orientation estimate");
            return gtsam::Rot3();
        }

        Eigen::Vector3d avg_accel = Eigen::Vector3d::Zero();
        size_t count = std::min(imu_buffer_.size(), params_.min_imu_samples_for_init);

        for (size_t i = 0; i < count; ++i)
        {
            avg_accel += imu_buffer_[i].accel;
        }
        avg_accel /= static_cast<double>(count);

        // DEBUG: Log average acceleration used for orientation estimation
        RCLCPP_INFO(get_logger(),
                    "DEBUG Initial orientation from %zu samples: avg_accel=(%.3f, %.3f, %.3f) |a|=%.3f",
                    count, avg_accel.x(), avg_accel.y(), avg_accel.z(), avg_accel.norm());

        // Accelerometer measures reaction to gravity, pointing UP when stationary
        // We want to find rotation R such that: R * avg_accel = [0, 0, +9.81] (world up)
        Eigen::Vector3d measured_up = avg_accel.normalized();
        Eigen::Vector3d world_up(0.0, 0.0, 1.0);

        // DEBUG: Log the vectors we're aligning
        RCLCPP_INFO(get_logger(),
                    "DEBUG Orientation: measured_up=(%.3f, %.3f, %.3f), world_up=(%.3f, %.3f, %.3f)",
                    measured_up.x(), measured_up.y(), measured_up.z(),
                    world_up.x(), world_up.y(), world_up.z());

        Eigen::Vector3d axis = measured_up.cross(world_up);
        double axis_norm = axis.norm();

        if (axis_norm < 1e-6)
        {
            // Vectors are parallel
            if (measured_up.dot(world_up) > 0)
            {
                RCLCPP_INFO(get_logger(), "DEBUG: IMU already aligned with world up");
                return gtsam::Rot3(); // Already aligned
            }
            else
            {
                RCLCPP_INFO(get_logger(), "DEBUG: IMU is upside down!");
                return gtsam::Rot3::Rx(M_PI); // Upside down
            }
        }

        axis.normalize();
        double angle = std::acos(std::clamp(measured_up.dot(world_up), -1.0, 1.0));

        RCLCPP_INFO(get_logger(),
                    "DEBUG Orientation: axis=(%.3f, %.3f, %.3f), angle=%.3f deg",
                    axis.x(), axis.y(), axis.z(), angle * 180.0 / M_PI);

        return gtsam::Rot3::AxisAngle(gtsam::Unit3(axis), angle);
    }

    // ============================================================================
    // State Creation & Optimization
    // ============================================================================

    void StateEstimator::create_new_state(const rclcpp::Time &timestamp)
    {
        // Caller must hold state_mutex_

        if (graph_states_.empty())
        {
            RCLCPP_ERROR(get_logger(), "Cannot create state: no previous state");
            return;
        }

        // Integrate IMU (needs imu_mutex_)
        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            integrate_imu_measurements_locked(last_state_time_, timestamp);
        }

        if (!preintegrated_imu_ || preintegrated_imu_->deltaTij() < 1e-6)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "No IMU data for state creation");
            return;
        }

        const auto &prev_state = graph_states_.back();
        uint64_t prev_key = prev_state.key_index;

        gtsam::NavState predicted;
        try
        {
            predicted = preintegrated_imu_->predict(prev_state.nav_state, prev_state.bias);

            // DEBUG: Detailed IMU prediction logging
            RCLCPP_INFO(get_logger(),
                        "DEBUG IMU predict: dt=%.3f, delta_pos=(%.3f, %.3f, %.3f), delta_vel=(%.3f, %.3f, %.3f)",
                        preintegrated_imu_->deltaTij(),
                        preintegrated_imu_->deltaPij().x(),
                        preintegrated_imu_->deltaPij().y(),
                        preintegrated_imu_->deltaPij().z(),
                        preintegrated_imu_->deltaVij().x(),
                        preintegrated_imu_->deltaVij().y(),
                        preintegrated_imu_->deltaVij().z());

            // DEBUG: Log bias values
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "DEBUG Current bias: accel=(%.4f, %.4f, %.4f), gyro=(%.4f, %.4f, %.4f)",
                                 prev_state.bias.accelerometer().x(),
                                 prev_state.bias.accelerometer().y(),
                                 prev_state.bias.accelerometer().z(),
                                 prev_state.bias.gyroscope().x(),
                                 prev_state.bias.gyroscope().y(),
                                 prev_state.bias.gyroscope().z());

            // DEBUG: Log predicted vs previous state
            RCLCPP_INFO(get_logger(),
                        "DEBUG State: prev_pos=(%.3f, %.3f, %.3f), pred_pos=(%.3f, %.3f, %.3f), delta_z=%.4f",
                        prev_state.nav_state.pose().x(), prev_state.nav_state.pose().y(), prev_state.nav_state.pose().z(),
                        predicted.pose().x(), predicted.pose().y(), predicted.pose().z(),
                        predicted.pose().z() - prev_state.nav_state.pose().z());

            // DEBUG: Log velocities
            RCLCPP_INFO(get_logger(),
                        "DEBUG Velocity: prev=(%.3f, %.3f, %.3f), pred=(%.3f, %.3f, %.3f)",
                        prev_state.nav_state.velocity().x(), prev_state.nav_state.velocity().y(), prev_state.nav_state.velocity().z(),
                        predicted.velocity().x(), predicted.velocity().y(), predicted.velocity().z());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "IMU prediction failed: %s", e.what());
            return;
        }

        uint64_t curr_key = current_key_index_ + 1;

        gtsam::NonlinearFactorGraph factors;
        gtsam::Values values;

        values.insert(X(curr_key), predicted.pose());
        values.insert(V(curr_key), predicted.velocity());
        values.insert(B(curr_key), prev_state.bias);

        // IMU factor
        factors.emplace_shared<gtsam::CombinedImuFactor>(
            X(prev_key), V(prev_key),
            X(curr_key), V(curr_key),
            B(prev_key), B(curr_key),
            *preintegrated_imu_);

        // GNSS factor with proper offset handling
        // GNSS factor with proper offset handling AND ANISOTROPIC NOISE
        // ANISOTROPIC GNSS NOISE: horizontal less accurate, vertical MORE accurate
        if (pending_gnss_.valid)
        {
            gtsam::Point3 corrected_pos = pending_gnss_.position;

            // Apply antenna offset if available
            if (gnss_to_base_)
            {
                gtsam::Pose3 current_pose_estimate = predicted.pose();
                gtsam::Point3 rotated_offset = current_pose_estimate.rotation().rotate(*gnss_to_base_);
                corrected_pos = corrected_pos - rotated_offset;
            }

            // CRITICAL: Trust GPS altitude much more than horizontal position
            gtsam::Vector3 gnss_sigmas;
            gnss_sigmas << pending_gnss_.sigma, // X (East) - higher uncertainty
                pending_gnss_.sigma,            // Y (North) - higher uncertainty
                params_.gnss_altitude_sigma;    // Z (Up) - MUCH LOWER uncertainty

            auto gnss_noise = gtsam::noiseModel::Diagonal::Sigmas(gnss_sigmas);
            gtsam::SharedNoiseModel noise;

            if (params_.use_robust_gnss_noise)
            {
                noise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Huber::Create(params_.gnss_huber_k),
                    gnss_noise);
            }
            else
            {
                noise = gnss_noise;
            }

            factors.emplace_shared<gtsam::GPSFactor>(X(curr_key), corrected_pos, noise);

            RCLCPP_DEBUG(get_logger(),
                         "Added GNSS factor at key %" PRIu64 ", sigma_xy=%.2f, sigma_z=%.2f",
                         curr_key, pending_gnss_.sigma, params_.gnss_altitude_sigma);
        }

        // Odometry factor
        if (pending_odom_.valid)
        {
            factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
                X(prev_key), X(curr_key), pending_odom_.delta, odom_noise_);
            RCLCPP_DEBUG(get_logger(), "Added odom factor %" PRIu64 "->%" PRIu64, prev_key, curr_key);
        }

        // Update ISAM2
        try
        {
            isam_->update(factors, values);
            auto result = isam_->calculateEstimate();

            GraphStateEntry entry;
            entry.key_index = curr_key;
            entry.timestamp = timestamp;
            entry.nav_state = gtsam::NavState(
                result.at<gtsam::Pose3>(X(curr_key)),
                result.at<gtsam::Vector3>(V(curr_key)));
            entry.bias = result.at<gtsam::imuBias::ConstantBias>(B(curr_key));

            // DEBUG: Log optimized state
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "DEBUG Optimized: pos=(%.3f, %.3f, %.3f), vel=(%.3f, %.3f, %.3f)",
                                 entry.nav_state.pose().x(), entry.nav_state.pose().y(), entry.nav_state.pose().z(),
                                 entry.nav_state.velocity().x(), entry.nav_state.velocity().y(), entry.nav_state.velocity().z());

            graph_states_.push_back(entry);
            current_key_index_ = curr_key;

            // Clear pending measurements only after successful update
            pending_gnss_.reset();
            pending_odom_.delta = gtsam::Pose3();
            pending_odom_.valid = false;
            // Keep prev_pose for accumulating next delta

            reset_preintegration(entry.bias);
            last_state_time_ = timestamp;

            {
                std::lock_guard<std::mutex> lock(imu_mutex_);
                prune_imu_buffer_locked(timestamp);
            }

            marginalize_old_states();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "ISAM2 update failed: %s", e.what());
            // Pending measurements are preserved for next attempt
        }
    }

    void StateEstimator::marginalize_old_states()
    {
        if (graph_states_.size() <= params_.max_graph_states)
        {
            return;
        }

        size_t num_to_remove = graph_states_.size() - params_.max_graph_states;

        gtsam::KeyList keys_to_remove;
        for (size_t i = 0; i < num_to_remove; ++i)
        {
            uint64_t key = graph_states_[i].key_index;
            keys_to_remove.push_back(X(key));
            keys_to_remove.push_back(V(key));
            keys_to_remove.push_back(B(key));
        }

        try
        {
            isam_->update(
                gtsam::NonlinearFactorGraph(),
                gtsam::Values(),
                gtsam::FactorIndices(),
                boost::none,
                boost::none,
                keys_to_remove);

            RCLCPP_DEBUG(get_logger(), "Marginalized %zu states", num_to_remove);
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(get_logger(), "Marginalization failed: %s", e.what());
        }

        graph_states_.erase(graph_states_.begin(), graph_states_.begin() + num_to_remove);

        if (!graph_states_.empty())
        {
            oldest_key_index_ = graph_states_.front().key_index;
        }
    }

    // ============================================================================
    // IMU Integration
    // ============================================================================

    void StateEstimator::integrate_imu_measurements_locked(
        const rclcpp::Time &from_time,
        const rclcpp::Time &to_time)
    {
        // Caller must hold imu_mutex_

        if (!preintegrated_imu_)
        {
            RCLCPP_ERROR(get_logger(), "Preintegration not initialized");
            return;
        }

        // Find first measurement after from_time
        auto it = imu_buffer_.begin();
        while (it != imu_buffer_.end() && it->timestamp <= from_time)
        {
            ++it;
        }

        if (it == imu_buffer_.end())
        {
            return;
        }

        // Use from_time as the starting reference for first integration
        rclcpp::Time prev_time = from_time;
        int integrated_count = 0;

        for (; it != imu_buffer_.end() && it->timestamp <= to_time; ++it)
        {
            double dt = (it->timestamp - prev_time).seconds();
            if (dt > 0.0 && dt < 1.0)
            {
                preintegrated_imu_->integrateMeasurement(it->accel, it->gyro, dt);
                integrated_count++;

                // DEBUG: Log first few integrations
                if (integrated_count <= 3)
                {
                    RCLCPP_DEBUG(get_logger(),
                                 "DEBUG Integrate[%d]: dt=%.4f, accel=(%.3f, %.3f, %.3f), gyro=(%.3f, %.3f, %.3f)",
                                 integrated_count, dt,
                                 it->accel.x(), it->accel.y(), it->accel.z(),
                                 it->gyro.x(), it->gyro.y(), it->gyro.z());
                }
            }
            prev_time = it->timestamp;
        }

        RCLCPP_DEBUG(get_logger(), "DEBUG: Integrated %d IMU measurements", integrated_count);
    }

    void StateEstimator::reset_preintegration(const gtsam::imuBias::ConstantBias &bias)
    {
        preintegrated_imu_ = std::make_unique<gtsam::PreintegratedCombinedMeasurements>(
            imu_params_, bias);
    }

    void StateEstimator::prune_imu_buffer_locked(const rclcpp::Time &before)
    {
        // Caller must hold imu_mutex_
        rclcpp::Time cutoff = before - rclcpp::Duration::from_seconds(0.1);

        while (!imu_buffer_.empty() && imu_buffer_.front().timestamp < cutoff)
        {
            imu_buffer_.pop_front();
        }
    }

    rclcpp::Time StateEstimator::get_imu_buffer_start_time() const
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        if (imu_buffer_.empty())
        {
            return rclcpp::Time(0, 0, RCL_ROS_TIME);
        }
        return imu_buffer_.front().timestamp;
    }

    rclcpp::Time StateEstimator::get_imu_buffer_end_time() const
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        if (imu_buffer_.empty())
        {
            return rclcpp::Time(0, 0, RCL_ROS_TIME);
        }
        return imu_buffer_.back().timestamp;
    }

    // ============================================================================
    // Coordinate Transforms
    // ============================================================================

    gtsam::Point3 StateEstimator::lla_to_enu(double lat, double lon, double alt)
    {
        if (!enu_origin_set_)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "lla_to_enu called but ENU origin not set, returning zero");
            return gtsam::Point3::Zero();
        }

        GeographicLib::LocalCartesian proj(origin_lat_, origin_lon_, origin_alt_);
        double x, y, z;
        proj.Forward(lat, lon, alt, x, y, z);

        return gtsam::Point3(x, y, z);
    }

    void StateEstimator::set_enu_origin(double lat, double lon, double alt)
    {
        origin_lat_ = lat;
        origin_lon_ = lon;
        origin_alt_ = alt;
        enu_origin_set_ = true;

        RCLCPP_INFO(get_logger(), "ENU origin: (%.6f, %.6f, %.2f)", lat, lon, alt);
    }

    // ============================================================================
    // TF Handling
    // ============================================================================

    void StateEstimator::lookup_sensor_transforms()
    {
        if (!imu_to_base_)
        {
            try
            {
                auto tf = tf_buffer_->lookupTransform(
                    params_.frames.base_frame, params_.frames.imu_frame,
                    tf2::TimePointZero, tf2::durationFromSec(0.0));

                imu_to_base_ = gtsam::Pose3(
                    gtsam::Rot3::Quaternion(
                        tf.transform.rotation.w,
                        tf.transform.rotation.x,
                        tf.transform.rotation.y,
                        tf.transform.rotation.z),
                    gtsam::Point3(
                        tf.transform.translation.x,
                        tf.transform.translation.y,
                        tf.transform.translation.z));

                RCLCPP_INFO(get_logger(), "Cached IMU->base transform");

                // DEBUG: Log the full transform
                RCLCPP_INFO(get_logger(),
                            "DEBUG IMU->base: trans=(%.3f, %.3f, %.3f), quat=(w=%.3f, x=%.3f, y=%.3f, z=%.3f)",
                            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z,
                            tf.transform.rotation.w, tf.transform.rotation.x,
                            tf.transform.rotation.y, tf.transform.rotation.z);
            }
            catch (const tf2::TransformException &)
            {
                // Not available yet
            }
        }

        if (!gnss_to_base_)
        {
            try
            {
                auto tf = tf_buffer_->lookupTransform(
                    params_.frames.base_frame, params_.frames.gnss_frame,
                    tf2::TimePointZero, tf2::durationFromSec(0.0));

                /*gnss_to_base_ = gtsam::Point3(
                    tf.transform.translation.x,
                    tf.transform.translation.y,
                    tf.transform.translation.z);*/
                gnss_to_base_ = gtsam::Point3(0, 0, 0); // Temporary: ignore lever arm

                RCLCPP_INFO(get_logger(), "Cached GNSS->base offset: (%.3f, %.3f, %.3f)",
                            gnss_to_base_->x(), gnss_to_base_->y(), gnss_to_base_->z());
            }
            catch (const tf2::TransformException &)
            {
                // Not available yet
            }
        }
    }

    void StateEstimator::publish_state()
    {
        auto state = get_latest_state();
        if (!state)
        {
            return;
        }

        odom_pub_->publish(state->to_odometry(cached_frames_.map, cached_frames_.base));
    }

    void StateEstimator::publish_transforms()
    {
        if (init_state_.load() != InitState::RUNNING)
        {
            return;
        }

        auto state = get_latest_state();
        if (!state)
        {
            return;
        }

        // Get odom->base transform from TF tree
        geometry_msgs::msg::TransformStamped odom_to_base;
        try
        {
            odom_to_base = tf_buffer_->lookupTransform(
                cached_frames_.odom, cached_frames_.base, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                                  "Cannot lookup odom->base: %s", ex.what());
            return;
        }

        // Convert to GTSAM Pose3
        gtsam::Pose3 T_odom_base(
            gtsam::Rot3::Quaternion(
                odom_to_base.transform.rotation.w,
                odom_to_base.transform.rotation.x,
                odom_to_base.transform.rotation.y,
                odom_to_base.transform.rotation.z),
            gtsam::Point3(
                odom_to_base.transform.translation.x,
                odom_to_base.transform.translation.y,
                odom_to_base.transform.translation.z));

        // Get estimated pose of base_link in map frame
        gtsam::Pose3 T_map_base = state->nav_state.pose();

        // Compute map->odom transform
        gtsam::Pose3 T_map_odom_raw = T_odom_base.between(T_map_base);

        // 180° rotation around Z-axis
        // This flips X and Y axes: X' = -X, Y' = -Y, Z' = Z
        gtsam::Rot3 R_z_180 = gtsam::Rot3::Rz(M_PI); // 180° around Z

        // Apply the rotation to the computed transform
        // Method A: Rotate the translation and orientation
        gtsam::Point3 rotated_translation = R_z_180.rotate(T_map_odom_raw.translation());
        gtsam::Rot3 rotated_orientation = R_z_180.compose(T_map_odom_raw.rotation());
        gtsam::Pose3 T_map_odom(rotated_orientation, rotated_translation);

        // Publish map->odom transform
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = state->timestamp;
        tf_msg.header.frame_id = cached_frames_.map;
        tf_msg.child_frame_id = cached_frames_.odom;

        tf_msg.transform.translation.x = T_map_odom.x();
        tf_msg.transform.translation.y = T_map_odom.y();
        tf_msg.transform.translation.z = T_map_odom.z();

        auto q = T_map_odom.rotation().toQuaternion();
        tf_msg.transform.rotation.w = q.w();
        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();

        tf_broadcaster_->sendTransform(tf_msg);

        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
                              "Published map->odom (Z-flipped): raw=(%.3f, %.3f, %.3f) -> flipped=(%.3f, %.3f, %.3f)",
                              T_map_odom_raw.x(), T_map_odom_raw.y(), T_map_odom_raw.z(),
                              T_map_odom.x(), T_map_odom.y(), T_map_odom.z());
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "DEBUG: When moving forward at 1 m/s:\n"
                             "  odom->base position: (%.2f, %.2f, %.2f)\n"
                             "  map->base position:  (%.2f, %.2f, %.2f)\n"
                             "  Both should increase in X if aligned correctly",
                             T_odom_base.x(), T_odom_base.y(), T_odom_base.z(),
                             T_map_base.x(), T_map_base.y(), T_map_base.z());
    }

    std::optional<EstimatedState> StateEstimator::get_latest_state() const
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        if (graph_states_.empty())
        {
            return std::nullopt;
        }

        const auto &latest = graph_states_.back();

        EstimatedState state;
        state.timestamp = latest.timestamp;
        state.nav_state = latest.nav_state;
        state.imu_bias = latest.bias;

        // Initialize with default uncertainty
        state.covariance = Eigen::Matrix<double, 15, 15>::Identity() * 0.1;

        try
        {
            // Pose covariance (indices 0-5)
            state.covariance.block<6, 6>(0, 0) = isam_->marginalCovariance(X(latest.key_index));

            // Velocity covariance (indices 6-8)
            state.covariance.block<3, 3>(6, 6) = isam_->marginalCovariance(V(latest.key_index));

            // Bias covariance (indices 9-14)
            state.covariance.block<6, 6>(9, 9) = isam_->marginalCovariance(B(latest.key_index));
        }
        catch (const std::exception &)
        {
            // Keep default covariance on failure
        }

        return state;
    }

} // namespace localizer

// ============================================================================
// Main
// ============================================================================

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<localizer::StateEstimator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}