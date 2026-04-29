// Safety watchdog node that stops the drone if commands or references timeout
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <chrono>

using namespace std::chrono_literals;

class SafetyWatchdog : public rclcpp::Node {
public:
  SafetyWatchdog()
  : Node("safety_watchdog"),
    last_cmd_time_(this->now()),
    last_ref_time_(this->now()),
    timeout_(0.4),
    stopped_(false),
    flying_(false)
  {
    sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/safe_bebop/cmd_vel", 10,
      std::bind(&SafetyWatchdog::cmdCallback, this, std::placeholders::_1));

    sub_ref_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/bebop/ref_vec", 10,
      std::bind(&SafetyWatchdog::refCallback, this, std::placeholders::_1));

    sub_is_flying_ = this->create_subscription<std_msgs::msg::Bool>(
      "/bebop/is_flying", 10,
      std::bind(&SafetyWatchdog::isFlyingCallback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/bebop/cmd_vel", 10);

    timer_ = this->create_wall_timer(50ms, std::bind(&SafetyWatchdog::checkTimeout, this));

    RCLCPP_INFO(this->get_logger(), "SafetyWatchdog started (timeout = %.2f s).", timeout_);
  }

private:
  void isFlyingCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    bool prev = flying_;
    flying_ = msg->data;
    if (flying_ && !prev) {
      RCLCPP_INFO(this->get_logger(), "is_flying TRUE — watchdog active.");
      last_cmd_time_ = this->now();
      last_ref_time_ = this->now();
      stopped_ = false;
    }
    else if (!flying_ && prev) {
      RCLCPP_INFO(this->get_logger(), "is_flying FALSE — watchdog inactive.");
      stopped_ = false;
      publishZero();
    }
  }

  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!flying_) return;
    last_cmd_time_ = this->now();
    if (!stopped_) {
      cmd_pub_->publish(*msg);
    }
  }

  void refCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (!flying_) return;
    if (msg->data.empty()) {
      RCLCPP_WARN(this->get_logger(), "Empty reference received — ignored.");
      return;
    }
    last_ref_time_ = this->now();
  }

  void checkTimeout() {
    if (!flying_) return;
    double dt_cmd = (this->now() - last_cmd_time_).seconds();
    double dt_ref = (this->now() - last_ref_time_).seconds();
    bool cmd_ok = dt_cmd <= timeout_;
    bool ref_ok = dt_ref <= timeout_;
    bool must_stop = !(cmd_ok && ref_ok);
    if (must_stop && !stopped_) {
      stopped_ = true;
      if (!cmd_ok && !ref_ok)
        RCLCPP_WARN(this->get_logger(), "Double timeout — STOP MODE");
      else if (!cmd_ok)
        RCLCPP_WARN(this->get_logger(), "cmd_vel timeout — STOP MODE");
      else
        RCLCPP_WARN(this->get_logger(), "ref_vec timeout — STOP MODE");
    }
    if (must_stop) {
      publishZero();
      return;
    }
    if (stopped_ && !must_stop) {
      stopped_ = false;
      RCLCPP_INFO(this->get_logger(), "Both flows restored — exiting STOP MODE.");
    }
  }

  void publishZero() {
    geometry_msgs::msg::Twist stop;
    stop.linear.x = stop.linear.y = stop.linear.z = stop.angular.z = 0.0;
    cmd_pub_->publish(stop);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_ref_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_is_flying_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_ref_time_;
  double timeout_;
  bool stopped_;
  bool flying_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyWatchdog>());
  rclcpp::shutdown();
  return 0;
}
