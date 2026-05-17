#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <geometry_msgs/msg/twist.hpp>
#include <iomanip>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>

namespace fs = std::filesystem;

class TopicLogger : public rclcpp::Node {
public:
  TopicLogger()
      : Node("topic_logger_node"), first_sample_(true), ref_first_sample_(true),
        ref_received_(false), odom_received_(false), cmd_first_sample_(true),
        yaw_cont_(0.0), yawd_cont_(0.0), yaw_rate_(0.0), yaw_rate_filt_(0.0),
        cmd_linx_raw_(0.0), cmd_liny_raw_(0.0), cmd_linz_raw_(0.0),
        cmd_angz_raw_(0.0), tau_yaw_rate_(0.08) {
    RCLCPP_INFO(this->get_logger(), "Logger node started");

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&TopicLogger::odomCallback, this, std::placeholders::_1));

    sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/bebop/cmd_vel", 10,
        std::bind(&TopicLogger::cmdCallback, this, std::placeholders::_1));

    sub_ref_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/bebop/ref_vec", 10,
        std::bind(&TopicLogger::refCallback, this, std::placeholders::_1));

    std::string base_path =
        std::string(std::getenv("HOME")) + "/ros2_ws/src/neroControl/data";
    fs::create_directories(base_path);

    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm = *std::localtime(&now_time);

    std::ostringstream filename;
    filename << base_path << "/bebop_log_"
             << std::put_time(&local_tm, "%Y%m%d_%H%M%S") << ".csv";

    logfile_.open(filename.str(), std::ios::out);
    if (!logfile_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Could not open log file: %s",
                   filename.str().c_str());
      throw std::runtime_error("Error opening CSV file");
    }

    RCLCPP_INFO(this->get_logger(), "Logging to: %s", filename.str().c_str());

    logfile_ << "time,"
             << "x,y,z,yaw,"
             << "linx_w,liny_w,linz_w,"
             << "linx_b,liny_b,linz_b,yaw_rate,yaw_rate_filt,"
             << "xd,yd,zd,yawd,"
             << "vxd_w,vyd_w,vzd_w,"
             << "vxd_b,vyd_b,vzd_b,wyawd,"
             << "cmd_linx,cmd_liny,cmd_linz,cmd_angz\n";

    timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / 30.0),
                                     std::bind(&TopicLogger::logData, this));
  }

  ~TopicLogger() { logfile_.close(); }

private:
  std::mutex mtx_;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    last_odom_ = *msg;
    odom_received_ = true;

    const auto &pose = msg->pose.pose;
    tf2::Quaternion q(pose.orientation.x, pose.orientation.y,
                      pose.orientation.z, pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    double t = rclcpp::Time(msg->header.stamp).seconds();

    if (!first_sample_) {
      double dyaw =
          std::atan2(std::sin(yaw - last_yaw_), std::cos(yaw - last_yaw_));
      yaw_cont_ += dyaw;

      double dt = t - last_time_;
      if (dt > 1e-3) {
        yaw_rate_ = dyaw / dt;
        double alpha = dt / (tau_yaw_rate_ + dt);
        yaw_rate_filt_ = (1.0 - alpha) * yaw_rate_filt_ + alpha * yaw_rate_;
      }
    } else {
      first_sample_ = false;
      yaw_cont_ = yaw;
      yaw_rate_ = 0.0;
      yaw_rate_filt_ = 0.0;
    }

    last_yaw_ = yaw;
    last_time_ = t;
  }

  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    cmd_linx_raw_ = msg->linear.x;
    cmd_liny_raw_ = msg->linear.y;
    cmd_linz_raw_ = msg->linear.z;
    cmd_angz_raw_ = msg->angular.z;
    last_cmd_ = *msg;
  }

  void refCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() == 8) {
      std::lock_guard<std::mutex> lock(mtx_);
      double yawd = msg->data[3];
      if (!ref_first_sample_) {
        double dyawd = std::atan2(std::sin(yawd - last_yawd_),
                                  std::cos(yawd - last_yawd_));
        yawd_cont_ += dyawd;
      } else {
        ref_first_sample_ = false;
        yawd_cont_ = yawd;
      }
      last_yawd_ = yawd;
      last_ref_ = msg->data;
      ref_received_ = true;
    }
  }

  void logData() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!odom_received_)
      return;

    double t = this->now().seconds();
    logfile_ << std::fixed << std::setprecision(6) << t << ",";

    logfile_ << last_odom_.pose.pose.position.x << ","
             << last_odom_.pose.pose.position.y << ","
             << last_odom_.pose.pose.position.z << "," << yaw_cont_ << ",";

    double lx_b = last_odom_.twist.twist.linear.x;
    double ly_b = last_odom_.twist.twist.linear.y;
    double lz_b = last_odom_.twist.twist.linear.z;
    double lx_w = lx_b * std::cos(last_yaw_) - ly_b * std::sin(last_yaw_);
    double ly_w = lx_b * std::sin(last_yaw_) + ly_b * std::cos(last_yaw_);
    double lz_w = lz_b;

    logfile_ << lx_w << "," << ly_w << "," << lz_w << "," << lx_b << "," << ly_b
             << "," << lz_b << "," << yaw_rate_ << "," << yaw_rate_filt_ << ",";

    if (ref_received_) {
      double vxd_w = last_ref_[4];
      double vyd_w = last_ref_[5];
      double vzd_w = last_ref_[6];
      double vxd_b =
          vxd_w * std::cos(last_yawd_) + vyd_w * std::sin(last_yawd_);
      double vyd_b =
          -vxd_w * std::sin(last_yawd_) + vyd_w * std::cos(last_yawd_);
      double vzd_b = vzd_w;

      logfile_ << last_ref_[0] << "," << last_ref_[1] << "," << last_ref_[2]
               << "," << yawd_cont_ << "," << vxd_w << "," << vyd_w << ","
               << vzd_w << "," << vxd_b << "," << vyd_b << "," << vzd_b << ","
               << last_ref_[7] << ",";
    } else {
      logfile_ << "0,0,0,0,0,0,0,0,0,0,0,";
    }

    logfile_ << cmd_linx_raw_ << "," << cmd_liny_raw_ << "," << cmd_linz_raw_
             << "," << cmd_angz_raw_ << "\n";
    logfile_.flush();
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_ref_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::ofstream logfile_;

  nav_msgs::msg::Odometry last_odom_;
  geometry_msgs::msg::Twist last_cmd_;
  std::vector<double> last_ref_;

  bool first_sample_, ref_first_sample_, ref_received_, odom_received_,
      cmd_first_sample_;
  double last_time_, last_yaw_, yaw_cont_, yaw_rate_, yaw_rate_filt_;
  double last_yawd_, yawd_cont_;
  double cmd_linx_raw_, cmd_liny_raw_, cmd_linz_raw_, cmd_angz_raw_;
  double tau_yaw_rate_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TopicLogger>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}