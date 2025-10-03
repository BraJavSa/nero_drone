#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

class TagTFDetector : public rclcpp::Node {
public:
    TagTFDetector() : Node("tag_tf_detector") {
        // Detector de AprilTags
        tf_ = tag36h11_create();
        td_ = apriltag_detector_create();
        apriltag_detector_add_family(td_, tf_);

        // Suscriptores
        sub_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/bebop/camera/camera_info", 10,
            std::bind(&TagTFDetector::camera_info_callback, this, std::placeholders::_1));

        sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/bebop/camera/image_raw", 10,
            std::bind(&TagTFDetector::image_callback, this, std::placeholders::_1));

        // Publicador Pose
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("tag_pose", 10);

        // TF broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        tag_size_ = 0.12; // metros
        RCLCPP_INFO(this->get_logger(), "tag_tf_detector node started.");
    }

    ~TagTFDetector() {
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
        RCLCPP_INFO(this->get_logger(), "Camera info received.");
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!has_camera_info_) return;

        cv::Mat frame;
        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (...) {
            return;
        }

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        image_u8_t im = { gray.cols, gray.rows, gray.cols, gray.data };
        zarray_t* detections = apriltag_detector_detect(td_, &im);

        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t* det;
            zarray_get(detections, i, &det);

            double s = tag_size_ / 2.0;
            std::vector<cv::Point3f> objectPoints = {
                {-s, -s, 0}, { s, -s, 0}, { s,  s, 0}, {-s,  s, 0}
            };

            std::vector<cv::Point2f> imagePoints = {
                {(float)det->p[0][0], (float)det->p[0][1]},
                {(float)det->p[1][0], (float)det->p[1][1]},
                {(float)det->p[2][0], (float)det->p[2][1]},
                {(float)det->p[3][0], (float)det->p[3][1]}
            };

            cv::Mat rvec, tvec;
            cv::solvePnP(objectPoints, imagePoints, cameraMatrix_, distCoeffs_, rvec, tvec);

            cv::Mat R;
            cv::Rodrigues(rvec, R);

            tf2::Quaternion q;
            cv::Matx33d mat(
                R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
                R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
                R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2)
            );
            double trace = mat(0,0) + mat(1,1) + mat(2,2);
            if (trace > 0.0) {
                double s = 0.5 / std::sqrt(trace + 1.0);
                q.setW(0.25 / s);
                q.setX((mat(2,1) - mat(1,2)) * s);
                q.setY((mat(0,2) - mat(2,0)) * s);
                q.setZ((mat(1,0) - mat(0,1)) * s);
            } else {
                q.setRPY(0,0,0);
            }

            // Publicar PoseStamped
            auto pose_msg = geometry_msgs::msg::PoseStamped();
            pose_msg.header.stamp = this->now();
            pose_msg.header.frame_id = "camera_optical";
            pose_msg.pose.position.x = tvec.at<double>(0);
            pose_msg.pose.position.y = tvec.at<double>(1);
            pose_msg.pose.position.z = tvec.at<double>(2);
            pose_msg.pose.orientation.x = q.x();
            pose_msg.pose.orientation.y = q.y();
            pose_msg.pose.orientation.z = q.z();
            pose_msg.pose.orientation.w = q.w();
            pose_pub_->publish(pose_msg);

            // Publicar TF del tag
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = this->now();
            tf_msg.header.frame_id = "camera_optical";
            tf_msg.child_frame_id = "tag_" + std::to_string(det->id);
            tf_msg.transform.translation.x = tvec.at<double>(0);
            tf_msg.transform.translation.y = tvec.at<double>(1);
            tf_msg.transform.translation.z = tvec.at<double>(2);
            tf_msg.transform.rotation.x = q.x();
            tf_msg.transform.rotation.y = q.y();
            tf_msg.transform.rotation.z = q.z();
            tf_msg.transform.rotation.w = q.w();
            tf_broadcaster_->sendTransform(tf_msg);
        }

        apriltag_detections_destroy(detections);
    }

    // ROS
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Detector
    apriltag_family_t *tf_;
    apriltag_detector_t *td_;

    // Calibración
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;
    bool has_camera_info_ = false;
    double tag_size_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TagTFDetector>());
    rclcpp::shutdown();
    return 0;
}
