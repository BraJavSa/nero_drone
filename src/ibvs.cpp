// ROS2 node for Image-Based Visual Servoing (IBVS) using AprilTags
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <cmath>
#include <vector>

class BebopTagNode : public rclcpp::Node {
public:
    BebopTagNode() : Node("bebop_tag_node") {
        tf_ = tag36h11_create();
        td_ = apriltag_detector_create();
        apriltag_detector_add_family(td_, tf_);
        pub_ref_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/bebop/ref_vec", 10);
        sub_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/bebop/camera/camera_info", 10,
            std::bind(&BebopTagNode::camera_info_callback, this, std::placeholders::_1));
        sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/bebop/camera/image_raw", 10,
            std::bind(&BebopTagNode::image_callback, this, std::placeholders::_1));
        tag_size_ = 0.165;
        has_prev_pose_ = false;
    }

    ~BebopTagNode() {
        apriltag_detector_destroy(td_);
        tag36h11_destroy(tf_);
    }

private:
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (has_camera_info_) return;
        cameraMatrix_ = (cv::Mat1d(3,3) <<
            msg->k[0], msg->k[1], msg->k[2],
            msg->k[3], msg->k[4], msg->k[5],
            msg->k[6], msg->k[7], msg->k[8]);
        distCoeffs_ = cv::Mat(msg->d).clone();
        has_camera_info_ = true;
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!has_camera_info_) return;
        cv::Mat frame;
        try { frame = cv_bridge::toCvCopy(msg, "bgr8")->image; }
        catch (...) { return; }
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        image_u8_t im = { gray.cols, gray.rows, gray.cols, gray.data };
        zarray_t* detections = apriltag_detector_detect(td_, &im);
        bool tag_detected = zarray_size(detections) > 0;
        if (tag_detected) {
            apriltag_detection_t* det;
            zarray_get(detections, 0, &det);
            double s = tag_size_ / 2.0;
            std::vector<cv::Point3f> objectPoints = {{-s, -s, 0}, {s, -s, 0}, {s, s, 0}, {-s, s, 0}};
            std::vector<cv::Point2f> imagePoints = {
                {float(det->p[0][0]), float(det->p[0][1])},
                {float(det->p[1][0]), float(det->p[1][1])},
                {float(det->p[2][0]), float(det->p[2][1])},
                {float(det->p[3][0]), float(det->p[3][1])}
            };
            cv::Mat rvec, tvec;
            cv::solvePnP(objectPoints, imagePoints, cameraMatrix_, distCoeffs_, rvec, tvec);
            float u = (imagePoints[0].x + imagePoints[1].x + imagePoints[2].x + imagePoints[3].x) / 4.0f;
            float v = (imagePoints[0].y + imagePoints[1].y + imagePoints[2].y + imagePoints[3].y) / 4.0f;
            float e_u = (frame.cols / 2.0f) - u;
            float e_v = (frame.rows / 2.0f) - v; 
            float dx = imagePoints[2].x - imagePoints[1].x;
            float dy = imagePoints[2].y - imagePoints[1].y;
            float e_yaw = 0.0f - std::atan2(dx, -dy);
            double e_z = 0.80 - tvec.at<double>(2);
            publish_velocity(e_u, e_v, e_z, e_yaw);
            visualize(frame, imagePoints, det->id, u, v);
        } else {
            publish_velocity(0, 0, 0, 0);
        }
        apriltag_detections_destroy(detections);
        cv::imshow("Bebop IBVS Control", frame);
        cv::waitKey(1);
    }

    void publish_velocity(float eu, float ev, double ez, float eyaw) {
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data.resize(8, 0.0);
        double Kp_v = 0.0005;
        double Kp_z = 0.5;
        double Kp_yaw = 0.2;
        msg.data[4] = ev * Kp_v;
        msg.data[5] = eu * Kp_v;
        msg.data[6] = ez * Kp_z;
        msg.data[7] = eyaw * Kp_yaw;
        pub_ref_->publish(msg);
    }

    void visualize(cv::Mat& frame, const std::vector<cv::Point2f>& pts, int id, float u, float v) {
        for (int j = 0; j < 4; j++)
            cv::line(frame, pts[j], pts[(j+1)%4], cv::Scalar(0,255,0), 2);
        cv::circle(frame, cv::Point(u, v), 5, cv::Scalar(0, 255, 255), -1);
    }

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_ref_;
    apriltag_family_t *tf_;
    apriltag_detector_t *td_;
    cv::Mat cameraMatrix_, distCoeffs_;
    double tag_size_;
    bool has_camera_info_ = false;
    bool has_prev_pose_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BebopTagNode>());
    rclcpp::shutdown();
    return 0;
}