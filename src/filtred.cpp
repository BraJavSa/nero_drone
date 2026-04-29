// ROS2 node for filtering reference vectors and tag positions using EMA
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class RefVecFilter : public rclcpp::Node {
public:
    RefVecFilter()
        : Node("ref_vec_filter_tf"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        ref_vec_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/bebop/ref_vec", 10);
        marker_pub_  = this->create_publisher<visualization_msgs::msg::Marker>("ref_marker", 10);

        sub_detected_ = this->create_subscription<std_msgs::msg::Bool>(
            "/bebop/detected", 10,
            std::bind(&RefVecFilter::detected_callback, this, std::placeholders::_1));

        alpha_pos_ = this->declare_parameter("alpha_pos", 0.93);
        alpha_vel_ = this->declare_parameter("alpha_vel", 0.75);
        first_detection_ = true;
        detected_ = false;

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&RefVecFilter::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "RefVecFilter started (30 Hz, alpha_pos=%.2f).", alpha_pos_);
    }

private:
    void detected_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        detected_ = msg->data;
    }

    void timer_callback() {
        if (!detected_) {
            publish_empty_markers();
            return;
        }

        geometry_msgs::msg::TransformStamped tf_tag_odom;
        try {
            tf_tag_odom = tf_buffer_.lookupTransform("odom", "tag_0", tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "TF odom->tag_0 not available: %s", ex.what());
            return;
        }

        double x = tf_tag_odom.transform.translation.x;
        double y = tf_tag_odom.transform.translation.y;
        double z = tf_tag_odom.transform.translation.z;
        double vx = 0.0;
        double vy = 0.0;
        double vz = 0.0; 

        if (first_detection_) {
            x_f_ = x; y_f_ = y; z_f_ = z;
            vx_f_ = vy_f_ = vz_f_ = 0.0;
            first_detection_ = false;
            last_time_ = this->now();
        } else {
            rclcpp::Time now = this->now();
            double dt = (now - last_time_).seconds();
            if (dt <= 0.0) return;
            last_time_ = now;

            vx = (x - x_prev_) / dt;
            vy = (y - y_prev_) / dt;
            vz = (z - z_prev_) / dt;

            x_f_ = alpha_pos_ * x + (1 - alpha_pos_) * x_f_;
            y_f_ = alpha_pos_ * y + (1 - alpha_pos_) * y_f_;
            z_f_ = alpha_pos_ * z + (1 - alpha_pos_) * z_f_;

            vx_f_ = alpha_vel_ * vx + (1 - alpha_vel_) * vx_f_;
            vy_f_ = alpha_vel_ * vy + (1 - alpha_vel_) * vy_f_;
            vz_f_ = alpha_vel_ * vz + (1 - alpha_vel_) * vz_f_;
        }

        x_prev_ = x; y_prev_ = y; z_prev_ = z;

        double x_ref = x - 2.0;
        double y_ref = y;
        double z_ref = 1.7;

        std_msgs::msg::Float64MultiArray ref_msg;
        ref_msg.data = {x_ref, y_ref, z_ref, 0.0, 0.0, 0.0, 0.0, 0.0};
        ref_vec_pub_->publish(ref_msg);

        publish_marker(0, "tag_filtered", x_f_, y_f_, z_f_, 1.0f, 0.0f, 0.0f);
        publish_marker(1, "ref_point", x_ref, y_ref, z_ref, 0.0f, 1.0f, 0.0f);
    }

    void publish_marker(int id, const std::string &ns,
                        double x, double y, double z,
                        float r, float g, float b)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = this->now();
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.08;
        marker.scale.y = 0.08;
        marker.scale.z = 0.08;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 0.9f;
        marker.lifetime = rclcpp::Duration(0, 0);
        marker_pub_->publish(marker);
    }

    void publish_empty_markers() {
        visualization_msgs::msg::Marker marker;
        marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_pub_->publish(marker);
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_detected_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ref_vec_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    bool first_detection_;
    bool detected_;
    double alpha_pos_, alpha_vel_;
    double x_f_, y_f_, z_f_;
    double vx_f_, vy_f_, vz_f_;
    double x_prev_, y_prev_, z_prev_;
    rclcpp::Time last_time_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RefVecFilter>());
    rclcpp::shutdown();
    return 0;
}
