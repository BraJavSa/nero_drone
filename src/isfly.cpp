// ROS2 node for tracking and publishing the drone's flight status (flying vs. grounded)
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class IsFlyNode : public rclcpp::Node {
public:
    IsFlyNode() : Node("isfly_node"), is_flying_(false) {
        sub_takeoff_ = this->create_subscription<std_msgs::msg::Empty>(
            "/bebop/takeoff", 10,
            std::bind(&IsFlyNode::takeoffCallback, this, std::placeholders::_1));

        sub_land_ = this->create_subscription<std_msgs::msg::Empty>(
            "/bebop/land", 10,
            std::bind(&IsFlyNode::landCallback, this, std::placeholders::_1));

        pub_is_flying_ = this->create_publisher<std_msgs::msg::Bool>("/bebop/is_flying", 10);
        timer_ = this->create_wall_timer(200ms, std::bind(&IsFlyNode::publishState, this));
        RCLCPP_INFO(this->get_logger(), "IsFlyNode started.");
    }

private:
    void takeoffCallback(const std_msgs::msg::Empty::SharedPtr) {
        if (is_flying_) {
            RCLCPP_WARN(this->get_logger(), "Takeoff ignored: drone is already flying.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Takeoff received. Waiting for lift-off...");
        std::thread([this]() {
            std::this_thread::sleep_for(3s);
            this->is_flying_ = true;
            RCLCPP_INFO(this->get_logger(), "Drone is now flying (is_flying = true).");
        }).detach();
    }

    void landCallback(const std_msgs::msg::Empty::SharedPtr) {
        if (!is_flying_) {
            RCLCPP_WARN(this->get_logger(), "Land ignored: drone is already grounded.");
            return;
        }
        is_flying_ = false;
        RCLCPP_INFO(this->get_logger(), "Drone has landed (is_flying = false).");
    }

    void publishState() {
        std_msgs::msg::Bool msg;
        msg.data = is_flying_;
        pub_is_flying_->publish(msg);
    }

    bool is_flying_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_takeoff_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_land_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_is_flying_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IsFlyNode>());
    rclcpp::shutdown();
    return 0;
}
