// C++ ROS 2 Node: Ground Truth Odometry generator from OptiTrack VRPN MoCap.
// Subscribes to /vrpn_mocap/bebop/pose (PoseStamped) with BEST_EFFORT QoS.
// Computes 15 Hz Odometry with Pose in Inertial Frame ('world') and Velocities in Body Frame ('bebop_gt').
// Broadcasts TF transform 'world' -> 'bebop_gt'.

#include <chrono>
#include <cmath>
#include <memory>
#include <deque>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

class CausalFilter {
public:
  CausalFilter(size_t window_size = 5, bool is_angle = false)
      : window_size_(window_size), is_angle_(is_angle) {}

  double update(double val) {
    buffer_.push_back(val);
    if (buffer_.size() > window_size_) {
      buffer_.pop_front();
    }

    if (is_angle_) {
      double sin_sum = 0.0;
      double cos_sum = 0.0;
      for (double v : buffer_) {
        sin_sum += std::sin(v);
        cos_sum += std::cos(v);
      }
      return std::atan2(sin_sum, cos_sum);
    }

    double sum = 0.0;
    for (double v : buffer_) {
      sum += v;
    }
    return sum / buffer_.size();
  }

private:
  size_t window_size_;
  bool is_angle_;
  std::deque<double> buffer_;
};

class GtMocapOdomCppNode : public rclcpp::Node {
public:
  GtMocapOdomCppNode()
      : Node("gt_mocap_odom_cpp_node"),
        x_filter_(5, false), y_filter_(5, false), z_filter_(5, false),
        roll_filter_(5, true), pitch_filter_(5, true), yaw_filter_(5, true),
        has_last_pose_(false), has_prev_state_(false) {

    // Declare ROS parameters
    this->declare_parameter<std::string>("mocap_topic", "/vrpn_mocap/bebop/pose");
    this->declare_parameter<std::string>("gt_odom_topic", "/bebop/gt_fullodom");
    this->declare_parameter<std::string>("alias_odom_topic", "/bebop/gt_odom");
    this->declare_parameter<double>("publish_rate", 15.0);
    this->declare_parameter<std::string>("world_frame", "world");
    this->declare_parameter<std::string>("child_frame", "bebop_gt");

    mocap_topic_ = this->get_parameter("mocap_topic").as_string();
    gt_odom_topic_ = this->get_parameter("gt_odom_topic").as_string();
    alias_odom_topic_ = this->get_parameter("alias_odom_topic").as_string();
    double rate_hz = this->get_parameter("publish_rate").as_double();
    world_frame_ = this->get_parameter("world_frame").as_string();
    child_frame_ = this->get_parameter("child_frame").as_string();

    RCLCPP_INFO(this->get_logger(), "Starting C++ GT Mocap Odom Node at %.1f Hz", rate_hz);
    RCLCPP_INFO(this->get_logger(), "Subscribing to: %s (BEST_EFFORT)", mocap_topic_.c_str());

    // Configure BEST_EFFORT QoS profile matching VRPN Mocap
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

    sub_mocap_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        mocap_topic_, qos,
        std::bind(&GtMocapOdomCppNode::mocapCallback, this, std::placeholders::_1));

    pub_gt_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(gt_odom_topic_, 10);
    pub_alias_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(alias_odom_topic_, 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    auto timer_period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = this->create_wall_timer(timer_period, std::bind(&GtMocapOdomCppNode::timerCallback, this));
  }

private:
  void mocapCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    last_pose_msg_ = msg;
    has_last_pose_ = true;
  }

  double angleDiff(double a, double b) {
    double d = a - b;
    while (d > M_PI) d -= 2.0 * M_PI;
    while (d < -M_PI) d += 2.0 * M_PI;
    return d;
  }

  void timerCallback() {
    if (!has_last_pose_) return;

    rclcpp::Time now = this->now();

    if (!has_prev_state_) {
      prev_stamp_ = now;
    }

    double dt = (now - prev_stamp_).seconds();
    if (dt <= 1e-5) dt = 1.0 / 15.0;
    prev_stamp_ = now;

    const auto &pos = last_pose_msg_->pose.position;
    const auto &ori = last_pose_msg_->pose.orientation;

    tf2::Quaternion q(ori.x, ori.y, ori.z, ori.w);
    double roll_raw, pitch_raw, yaw_raw;
    tf2::Matrix3x3(q).getRPY(roll_raw, pitch_raw, yaw_raw);

    // Apply causal filters
    double x_f = x_filter_.update(pos.x);
    double y_f = y_filter_.update(pos.y);
    double z_f = z_filter_.update(pos.z);
    double roll_f = roll_filter_.update(roll_raw);
    double pitch_f = pitch_filter_.update(pitch_raw);
    double yaw_f = yaw_filter_.update(yaw_raw);

    double vx_body = 0.0, vy_body = 0.0, vz_body = 0.0, wz_body = 0.0;

    if (has_prev_state_) {
      double dx_w = (x_f - prev_x_) / dt;
      double dy_w = (y_f - prev_y_) / dt;
      double dz_w = (z_f - prev_z_) / dt;
      double dyaw_w = angleDiff(yaw_f, prev_yaw_) / dt;

      // 2D Rotation (Inertial velocity to Body velocity)
      double cy = std::cos(yaw_f);
      double sy = std::sin(yaw_f);

      vx_body = cy * dx_w + sy * dy_w;
      vy_body = -sy * dx_w + cy * dy_w;
      vz_body = dz_w;
      wz_body = dyaw_w;
    } else {
      has_prev_state_ = true;
    }

    prev_x_ = x_f;
    prev_y_ = y_f;
    prev_z_ = z_f;
    prev_yaw_ = yaw_f;

    // Construct Odometry Message
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = world_frame_;
    odom.child_frame_id = child_frame_;

    odom.pose.pose.position.x = x_f;
    odom.pose.pose.position.y = y_f;
    odom.pose.pose.position.z = z_f;
    odom.pose.pose.orientation = ori;

    // High precision Mocap covariance
    for (int i = 0; i < 36; ++i) odom.pose.covariance[i] = 0.0;
    odom.pose.covariance[0] = 1e-5;
    odom.pose.covariance[7] = 1e-5;
    odom.pose.covariance[14] = 1e-5;
    odom.pose.covariance[21] = 1e-4;
    odom.pose.covariance[28] = 1e-4;
    odom.pose.covariance[35] = 1e-4;

    odom.twist.twist.linear.x = vx_body;
    odom.twist.twist.linear.y = vy_body;
    odom.twist.twist.linear.z = vz_body;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = wz_body;

    for (int i = 0; i < 36; ++i) odom.twist.covariance[i] = 0.0;
    odom.twist.covariance[0] = 1e-3;
    odom.twist.covariance[7] = 1e-3;
    odom.twist.covariance[14] = 1e-3;
    odom.twist.covariance[35] = 1e-3;

    pub_gt_odom_->publish(odom);
    pub_alias_odom_->publish(odom);

    // Broadcast TF
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = now;
    tf_msg.header.frame_id = world_frame_;
    tf_msg.child_frame_id = child_frame_;
    tf_msg.transform.translation.x = x_f;
    tf_msg.transform.translation.y = y_f;
    tf_msg.transform.translation.z = z_f;
    tf_msg.transform.rotation = ori;

    tf_broadcaster_->sendTransform(tf_msg);
  }

  std::string mocap_topic_, gt_odom_topic_, alias_odom_topic_;
  std::string world_frame_, child_frame_;

  CausalFilter x_filter_, y_filter_, z_filter_;
  CausalFilter roll_filter_, pitch_filter_, yaw_filter_;

  geometry_msgs::msg::PoseStamped::SharedPtr last_pose_msg_;
  bool has_last_pose_, has_prev_state_;

  rclcpp::Time prev_stamp_;
  double prev_x_, prev_y_, prev_z_, prev_yaw_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_mocap_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_gt_odom_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_alias_odom_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GtMocapOdomCppNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
