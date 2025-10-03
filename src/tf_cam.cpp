#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <algorithm>

class CameraTFController : public rclcpp::Node {
public:
    CameraTFController() : Node("camera_tf_controller") {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Sub al comando de cámara
        sub_cmd_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/bebop/move_camera", 10,
            std::bind(&CameraTFController::cmdCallback, this, std::placeholders::_1));

        // Timer para control (50 Hz = 20 ms)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&CameraTFController::update, this));

        // Inicializar en cero
        tilt_target_ = 0.0;
        pan_target_  = 0.0;
        tilt_current_ = 0.0;
        pan_current_  = 0.0;

        delay_timer_ = 0.0;

        RCLCPP_INFO(this->get_logger(), "Camera TF Controller node started.");
    }

private:
    void cmdCallback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        // Guardar referencias deseadas, con saturación
        tilt_target_ = std::clamp(msg->x, -90.0, 10.0);   // vertical
        pan_target_  = std::clamp(msg->y, -70.0, 70.0);   // horizontal

        delay_timer_ = 0.8; // 1 segundo de espera antes de mover
        RCLCPP_INFO(this->get_logger(),
            "New target -> Tilt: %.1f°, Pan: %.1f° (after 1s delay)",
            tilt_target_, pan_target_);
    }

    void update() {
        double dt = 0.005; // 50 Hz

        if (delay_timer_ > 0.0) {
            delay_timer_ -= dt;
            return; // aún no comienza el movimiento
        }

        // Velocidades máximas (deg/s)
        double max_tilt_speed = 100.0 / 5.0; 
        double max_pan_speed  = 140.0 / 5.0; 

        // --- Tilt ---
        double tilt_error = tilt_target_ - tilt_current_;
        double tilt_step = std::clamp(tilt_error, -max_tilt_speed * dt, max_tilt_speed * dt);
        tilt_current_ += tilt_step;

        // --- Pan ---
        double pan_error = pan_target_ - pan_current_;
        double pan_step = std::clamp(pan_error, -max_pan_speed * dt, max_pan_speed * dt);
        pan_current_ += pan_step;

        // Publicar TF dinámico base_link -> camera_optical
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = this->now();
        tf_msg.header.frame_id = "base_link";
        tf_msg.child_frame_id = "camera_optical";
        tf_msg.transform.translation.x = 0.09; // cámara 9 cm delante
        tf_msg.transform.translation.y = 0.0;
        tf_msg.transform.translation.z = 0.02;

        // Convertir grados a radianes
        double tilt_rad = tilt_current_ * M_PI / 180.0;
        double pan_rad  = pan_current_  * M_PI / 180.0;

        // Nota: ajusta ejes según tu configuración física
        tf2::Quaternion q;
        q.setRPY(-M_PI/2 + tilt_rad, 0.0, -M_PI/2 + pan_rad);
        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tf_msg.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(tf_msg);
    }

    // Subs y pubs
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_cmd_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Estados
    double tilt_target_, pan_target_;
    double tilt_current_, pan_current_;
    double delay_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraTFController>());
    rclcpp::shutdown();
    return 0;
}
