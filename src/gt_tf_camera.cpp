// Computes and broadcasts virtual gimbal and camera coordinate transformations using GT frame (bebop_gt).

#include <algorithm>
#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

using namespace std::chrono_literals;

class GtCameraTFWithGimbal : public rclcpp::Node {
public:
  GtCameraTFWithGimbal() : Node("gt_camera_tf_with_gimbal") {
    this->declare_parameter("world_frame", "world");
    this->declare_parameter("child_frame", "bebop_gt");

    world_frame_ = this->get_parameter("world_frame").as_string();
    child_frame_ = this->get_parameter("child_frame").as_string();

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    sub_move_camera_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        "/bebop/move_camera", 10,
        std::bind(&GtCameraTFWithGimbal::move_camera_callback, this,
                  std::placeholders::_1));

    pub_camera_moving_ =
        this->create_publisher<std_msgs::msg::Bool>("/bebop/camera_moving", 10);

    timer_ = this->create_wall_timer(
        20ms, std::bind(&GtCameraTFWithGimbal::update_camera_pitch, this));

    min_pitch_deg_ = -90.0;
    max_pitch_deg_ = 15.0;
    min_pitch_ = min_pitch_deg_ * M_PI / 180.0;
    max_pitch_ = max_pitch_deg_ * M_PI / 180.0;
    angular_speed_ = 22.0 * M_PI / 180.0;
    delay_before_move_ = 0.5;
    tilt_offset_ = -10.0 * M_PI / 180.0;
    gimbal_abs_max_ = 20.0 * M_PI / 180.0;
    gimbal_abs_min_ = -100.0 * M_PI / 180.0;
    current_pitch_ = 0.0;
    target_pitch_ = 0.0;
    waiting_ = false;
    moving_ = false;
    extra_publish_duration_ = 2.0;
    movement_end_time_ = this->now();
    last_update_time_ = this->now();
    wait_start_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "GtCameraTFWithGimbal started for frame: %s.", child_frame_.c_str());
  }

private:
  void move_camera_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    double target_deg = std::clamp(msg->x, min_pitch_deg_, max_pitch_deg_);
    target_pitch_ = target_deg * M_PI / 180.0;
    waiting_ = true;
    moving_ = true;
    wait_start_time_ = this->now();
  }

  void update_camera_pitch() {
    auto now = this->now();
    double dt = (now - last_update_time_).seconds();
    last_update_time_ = now;

    if (waiting_) {
      if ((now - wait_start_time_).seconds() >= delay_before_move_)
        waiting_ = false;
    }

    if (moving_ && !waiting_) {
      double error = target_pitch_ - current_pitch_;
      double direction = (error > 0.0) ? 1.0 : -1.0;
      double delta = direction * angular_speed_ * dt;
      if (std::fabs(error) <= std::fabs(delta)) {
        current_pitch_ = target_pitch_;
        moving_ = false;
        movement_end_time_ = now;
      } else {
        current_pitch_ += delta;
      }
    }

    geometry_msgs::msg::TransformStamped tf_base;
    try {
      tf_base = tf_buffer_->lookupTransform(
          world_frame_, child_frame_, tf2::TimePointZero,
          tf2::durationFromSec(0.01)
      );
    } catch (const tf2::TransformException &ex) {
      try {
        tf_base = tf_buffer_->lookupTransform(
            "odom", child_frame_, tf2::TimePointZero,
            tf2::durationFromSec(0.01)
        );
      } catch (const tf2::TransformException &ex2) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "lookupTransform %s->%s failed: %s",
                             world_frame_.c_str(), child_frame_.c_str(), ex.what());
        return;
      }
    }

    tf2::Quaternion q_drone(
        tf_base.transform.rotation.x, tf_base.transform.rotation.y,
        tf_base.transform.rotation.z, tf_base.transform.rotation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q_drone).getRPY(roll, pitch, yaw);

    double gimbal_max = gimbal_abs_max_ - current_pitch_;
    double gimbal_min = gimbal_abs_min_ - current_pitch_;
    double limited_pitch = std::clamp(pitch, gimbal_min, gimbal_max);

    tf2::Quaternion q_pitch_comp, q_roll_comp, q_total_comp;
    q_pitch_comp.setRPY(0.0, -limited_pitch, 0.0);
    q_roll_comp.setRPY(-roll, 0.0, 0.0);
    q_total_comp = q_roll_comp * q_pitch_comp;
    q_total_comp.normalize();

    publish_all_tfs(current_pitch_, q_total_comp);

    bool is_moving =
        waiting_ || moving_ ||
        (now - movement_end_time_).seconds() < extra_publish_duration_;
    publish_camera_moving(is_moving);
  }

  void publish_camera_moving(bool is_moving) {
    std_msgs::msg::Bool msg;
    msg.data = is_moving;
    pub_camera_moving_->publish(msg);
  }

  void publish_all_tfs(double pitch_angle,
                       const tf2::Quaternion &q_gimbal_comp) {
    auto stamp = this->get_clock()->now();
    std::vector<geometry_msgs::msg::TransformStamped> tfs(3);

    auto &tf_abs = tfs[0];
    tf_abs.header.stamp = stamp;
    tf_abs.header.frame_id = child_frame_;
    tf_abs.child_frame_id = "absolute_cam_link";
    tf_abs.transform.translation.x = 0.12;
    tf_abs.transform.translation.y = 0.0;
    tf_abs.transform.translation.z = 0.01;
    {
      tf2::Quaternion q_tilt;
      q_tilt.setRPY(0.0, -tilt_offset_, 0.0);
      tf_abs.transform.rotation.x = q_tilt.x();
      tf_abs.transform.rotation.y = q_tilt.y();
      tf_abs.transform.rotation.z = q_tilt.z();
      tf_abs.transform.rotation.w = q_tilt.w();
    }

    auto &tf_cam = tfs[1];
    tf_cam.header.stamp = stamp;
    tf_cam.header.frame_id = child_frame_;
    tf_cam.child_frame_id = "camera_link";
    tf_cam.transform.translation.x = 0.12;
    tf_cam.transform.translation.y = 0.0;
    tf_cam.transform.translation.z = 0.02;
    {
      tf2::Quaternion q_pitch;
      q_pitch.setRPY(0.0, -pitch_angle, 0.0);
      tf_cam.transform.rotation.x = q_pitch.x();
      tf_cam.transform.rotation.y = q_pitch.y();
      tf_cam.transform.rotation.z = q_pitch.z();
      tf_cam.transform.rotation.w = q_pitch.w();
    }

    auto &tf_gimbal = tfs[2];
    tf_gimbal.header.stamp = stamp;
    tf_gimbal.header.frame_id = "camera_link";
    tf_gimbal.child_frame_id = "camera_gimbal";
    tf_gimbal.transform.translation.x = 0.0;
    tf_gimbal.transform.translation.y = 0.0;
    tf_gimbal.transform.translation.z = 0.0;
    tf_gimbal.transform.rotation.x = q_gimbal_comp.x();
    tf_gimbal.transform.rotation.y = q_gimbal_comp.y();
    tf_gimbal.transform.rotation.z = q_gimbal_comp.z();
    tf_gimbal.transform.rotation.w = q_gimbal_comp.w();

    tf_broadcaster_->sendTransform(tfs);
  }

  std::string world_frame_, child_frame_;
  double min_pitch_, max_pitch_, min_pitch_deg_, max_pitch_deg_;
  double current_pitch_, target_pitch_, angular_speed_, delay_before_move_,
      tilt_offset_;
  double gimbal_abs_max_, gimbal_abs_min_;
  bool waiting_, moving_;
  double extra_publish_duration_;
  rclcpp::Time movement_end_time_, last_update_time_, wait_start_time_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_move_camera_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_camera_moving_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GtCameraTFWithGimbal>());
  rclcpp::shutdown();
  return 0;
}
