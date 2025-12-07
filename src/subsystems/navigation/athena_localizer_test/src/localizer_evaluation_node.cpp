#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <fstream>
#include <vector>

class LocalizerEvaluationNode : public rclcpp::Node
{
public:
  LocalizerEvaluationNode() : Node("localizer_evaluation_node"), alignment_set_(false)
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
      std::bind(&LocalizerEvaluationNode::evaluate, this));

    output_.open("evaluation.csv");
    output_ << "timestamp,trans_error,rot_error,x,y,z,roll,pitch,yaw\n";
  }

  ~LocalizerEvaluationNode()
  {
    if (output_.is_open()) output_.close();
    print_stats();
  }

private:
  void evaluate()
  {
    geometry_msgs::msg::TransformStamped gt, est;

    try {
      gt = tf_buffer_->lookupTransform("world", "base_link", tf2::TimePointZero);
      est = tf_buffer_->lookupTransform("test_jawn/map", "test_jawn/base_link", tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
      return;
    }

    if (!alignment_set_) {
      offset_x_ = gt.transform.translation.x - est.transform.translation.x;
      offset_y_ = gt.transform.translation.y - est.transform.translation.y;
      offset_z_ = gt.transform.translation.z - est.transform.translation.z;

      tf2::Quaternion gt_q(gt.transform.rotation.x, gt.transform.rotation.y,
                           gt.transform.rotation.z, gt.transform.rotation.w);
      tf2::Quaternion est_q(est.transform.rotation.x, est.transform.rotation.y,
                            est.transform.rotation.z, est.transform.rotation.w);

      offset_rot_ = gt_q * est_q.inverse();
      alignment_set_ = true;
    }

    tf2::Vector3 est_pos(est.transform.translation.x + offset_x_,
                         est.transform.translation.y + offset_y_,
                         est.transform.translation.z + offset_z_);

    tf2::Quaternion est_q(est.transform.rotation.x, est.transform.rotation.y,
                          est.transform.rotation.z, est.transform.rotation.w);
    tf2::Quaternion est_rot = offset_rot_ * est_q;

    double dx = gt.transform.translation.x - est_pos.x();
    double dy = gt.transform.translation.y - est_pos.y();
    double dz = gt.transform.translation.z - est_pos.z();
    double trans_error = std::sqrt(dx*dx + dy*dy + dz*dz);

    tf2::Quaternion gt_q(gt.transform.rotation.x, gt.transform.rotation.y,
                         gt.transform.rotation.z, gt.transform.rotation.w);

    double gt_r, gt_p, gt_y, est_r, est_p, est_y;
    tf2::Matrix3x3(gt_q).getRPY(gt_r, gt_p, gt_y);
    tf2::Matrix3x3(est_rot).getRPY(est_r, est_p, est_y);

    double dr = normalize(gt_r - est_r) * 180.0 / M_PI;
    double dp = normalize(gt_p - est_p) * 180.0 / M_PI;
    double dy_angle = normalize(gt_y - est_y) * 180.0 / M_PI;

    tf2::Quaternion error_q = gt_q * est_rot.inverse();
    double rot_error = 2.0 * std::acos(std::min(1.0, std::abs(error_q.w()))) * 180.0 / M_PI;

    double timestamp = gt.header.stamp.sec + gt.header.stamp.nanosec * 1e-9;

    output_ << timestamp << "," << trans_error << "," << rot_error << ","
            << dx << "," << dy << "," << dz << ","
            << dr << "," << dp << "," << dy_angle << "\n";

    trans_errors_.push_back(trans_error);
    rot_errors_.push_back(rot_error);
  }

  double normalize(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  void print_stats()
  {
    if (trans_errors_.empty()) return;

    double trans_sum = 0, rot_sum = 0, trans_max = 0, rot_max = 0;
    double trans_sq = 0, rot_sq = 0;

    for (size_t i = 0; i < trans_errors_.size(); ++i) {
      trans_sum += trans_errors_[i];
      rot_sum += rot_errors_[i];
      trans_sq += trans_errors_[i] * trans_errors_[i];
      rot_sq += rot_errors_[i] * rot_errors_[i];
      trans_max = std::max(trans_max, trans_errors_[i]);
      rot_max = std::max(rot_max, rot_errors_[i]);
    }

    size_t n = trans_errors_.size();

    RCLCPP_INFO(this->get_logger(), "\n========== EVALUATION STATISTICS ==========");
    RCLCPP_INFO(this->get_logger(), "Samples: %zu", n);
    RCLCPP_INFO(this->get_logger(), "Translation: Mean %.4f m | RMSE %.4f m | Max %.4f m",
                trans_sum/n, std::sqrt(trans_sq/n), trans_max);
    RCLCPP_INFO(this->get_logger(), "Rotation: Mean %.4f deg | RMSE %.4f deg | Max %.4f deg",
                rot_sum/n, std::sqrt(rot_sq/n), rot_max);
    RCLCPP_INFO(this->get_logger(), "==========================================\n");
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::ofstream output_;
  std::vector<double> trans_errors_, rot_errors_;
  bool alignment_set_;
  double offset_x_, offset_y_, offset_z_;
  tf2::Quaternion offset_rot_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizerEvaluationNode>());
  rclcpp::shutdown();
  return 0;
}
