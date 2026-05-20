// Computes the coordinate transform between the detected AprilTag and the Bebop base link.


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class TagTFDetector : public rclcpp::Node {
public:
    TagTFDetector()
    : Node("tag_tf_detector"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_),
      tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this))
    {
        tf_ = tag36h11_create();
        td_ = apriltag_detector_create();
        apriltag_detector_add_family(td_, tf_);

        sub_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/bebop/camera/camera_info", 10,
            std::bind(&TagTFDetector::camera_info_callback, this, std::placeholders::_1));

        sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/bebop/camera/image_raw", 10,
            std::bind(&TagTFDetector::image_callback, this, std::placeholders::_1));

        detected_pub_ = this->create_publisher<std_msgs::msg::Bool>("/bebop/detected", 10);

        tag_size_ = 0.145;
        alpha_pos_ = 0.45;
        alpha_rot_ = 0.4;
        first_detection_ = true;

        RCLCPP_INFO(this->get_logger(), "TagTFDetector started.");
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
        try { frame = cv_bridge::toCvCopy(msg, "bgr8")->image; }
        catch (...) { return; }
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        image_u8_t im = { gray.cols, gray.rows, gray.cols, gray.data };
        zarray_t* detections = apriltag_detector_detect(td_, &im);
        bool tag_detected = false;
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t* det;
            zarray_get(detections, i, &det);
            if (det->id != 0) continue;
            tag_detected = true;
            double s = tag_size_ / 2.0;
            std::vector<cv::Point3f> objectPoints = {{-s,-s,0}, {s,-s,0}, {s,s,0}, {-s,s,0}};
            std::vector<cv::Point2f> imagePoints = {
                {(float)det->p[0][0], (float)det->p[0][1]},
                {(float)det->p[1][0], (float)det->p[1][1]},
                {(float)det->p[2][0], (float)det->p[2][1]},
                {(float)det->p[3][0], (float)det->p[3][1]}
            };
            cv::Mat rvec, tvec;
            cv::solvePnP(objectPoints, imagePoints, cameraMatrix_, distCoeffs_, rvec, tvec);
            cv::Mat R_tag2cam;
            cv::Rodrigues(rvec, R_tag2cam);
            cv::Mat R_y180 = (cv::Mat1d(3,3) << -1,0,0, 0,1,0, 0,0,-1);
            cv::Mat R_z180 = (cv::Mat1d(3,3) << -1,0,0, 0,-1,0, 0,0,1);
            cv::Mat R_fix  = (cv::Mat1d(3,3) << 1,0,0, 0,-1,0, 0,0,-1);
            cv::Mat R_corrected = R_fix * R_z180 * R_y180 * R_tag2cam;
            tf2::Matrix3x3 tf2_rot(
                R_corrected.at<double>(0,0), R_corrected.at<double>(0,1), R_corrected.at<double>(0,2),
                R_corrected.at<double>(1,0), R_corrected.at<double>(1,1), R_corrected.at<double>(1,2),
                R_corrected.at<double>(2,0), R_corrected.at<double>(2,1), R_corrected.at<double>(2,2)
            );
            tf2::Quaternion q_meas;
            tf2_rot.getRotation(q_meas);
            q_meas.normalize();
            if (first_detection_) {
                pos_filt_.x = tvec.at<double>(0);
                pos_filt_.y = tvec.at<double>(1);
                pos_filt_.z = tvec.at<double>(2);
                q_filt_ = q_meas;
                first_detection_ = false;
            } else {
                pos_filt_.x = alpha_pos_ * tvec.at<double>(0) + (1 - alpha_pos_) * pos_filt_.x;
                pos_filt_.y = alpha_pos_ * tvec.at<double>(1) + (1 - alpha_pos_) * pos_filt_.y;
                pos_filt_.z = alpha_pos_ * tvec.at<double>(2) + (1 - alpha_pos_) * pos_filt_.z;
                q_filt_ = q_filt_.slerp(q_meas, alpha_rot_);
                q_filt_.normalize();
            }
            geometry_msgs::msg::PoseStamped pose_cam;
            pose_cam.header.frame_id = "camera_gimbal";
            pose_cam.header.stamp = this->now();
            pose_cam.pose.position = pos_filt_;
            pose_cam.pose.orientation.x = q_filt_.x();
            pose_cam.pose.orientation.y = q_filt_.y();
            pose_cam.pose.orientation.z = q_filt_.z();
            pose_cam.pose.orientation.w = q_filt_.w();
            try {
                geometry_msgs::msg::TransformStamped tf_cam_to_odom = tf_buffer_.lookupTransform("odom", "camera_gimbal", tf2::TimePointZero);
                geometry_msgs::msg::PoseStamped pose_odom;
                tf2::doTransform(pose_cam, pose_odom, tf_cam_to_odom);
                geometry_msgs::msg::TransformStamped tf_tag_odom;
                tf_tag_odom.header.stamp = this->now();
                tf_tag_odom.header.frame_id = "odom";
                tf_tag_odom.child_frame_id = "tag_0";
                tf_tag_odom.transform.translation.x = pose_odom.pose.position.x;
                tf_tag_odom.transform.translation.y = pose_odom.pose.position.y;
                tf_tag_odom.transform.translation.z = pose_odom.pose.position.z;
                tf_tag_odom.transform.rotation = pose_odom.pose.orientation;
                tf_broadcaster_->sendTransform(tf_tag_odom);
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Transform error: %s", ex.what());
            }
        }
        std_msgs::msg::Bool det;
        det.data = tag_detected;
        detected_pub_->publish(det);
        apriltag_detections_destroy(detections);
    }

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detected_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    apriltag_family_t *tf_;
    apriltag_detector_t *td_;
    cv::Mat cameraMatrix_, distCoeffs_;
    bool has_camera_info_ = false;
    bool first_detection_;
    double alpha_pos_, alpha_rot_;
    geometry_msgs::msg::Point pos_filt_;
    tf2::Quaternion q_filt_;
    double tag_size_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TagTFDetector>());
    rclcpp::shutdown();
    return 0;
}
