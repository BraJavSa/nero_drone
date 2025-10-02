#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("hello_cpp");
    RCLCPP_INFO(node->get_logger(), "Hola mundo desde C++ en nero_drone!");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
