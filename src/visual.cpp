// ROS2 node for AprilTag detection, TF broadcasting, and visual overlay for Bebop drone
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <opencv2/opencv.hpp>

extern "C" {
    #include <apriltag/apriltag.h>
    #include <apriltag/tag36h11.h>
}

class BebopTagNode : public rclcpp::Node {
public:
    BebopTagNode() : Node("bebop_tag_node"), has_camera_info_(false) {
        tf_ = tag36h11_create();
        td_ = apriltag_detector_create();
        apriltag_detector_add_family(td_, tf_);
        pub_detected_ = this->create_publisher<std_msgs::msg::Bool>("/bebop/detected", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        sub_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/bebop/camera/camera_info", 10,
            std::bind(&BebopTagNode::camera_info_callback, this, std::placeholders::_1));
        sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/bebop/camera/image_raw", 10,
            std::bind(&BebopTagNode::image_callback, this, std::placeholders::_1));
        tag_size_ = 0.12; 
        cv::namedWindow("Bebop Camera", cv::WINDOW_AUTOSIZE);
        RCLCPP_INFO(this->get_logger(), "Bebop AprilTag Node initialized.");
    }

    ~BebopTagNode() {
        apriltag_detector_destroy(td_);
        tag36h11_destroy(tf_);
        cv::destroyAllWindows();
    }

private:
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (has_camera_info_) return;
        cameraMatrix_ = (cv::Mat1d(3, 3) << 
            msg->k[0], msg->k[1], msg->k[2],
            msg->k[3], msg->k[4], msg->k[5],
            msg->k[6], msg->k[7], msg->k[8]);
        distCoeffs_ = cv::Mat(msg->d).clone();
        has_camera_info_ = true;
        RCLCPP_INFO(this->get_logger(), "Camera info received.");
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat frame;
        try { frame = cv_bridge::toCvCopy(msg, "bgr8")->image; }
        catch (cv_bridge::Exception& e) { return; }
        if (!has_camera_info_) {
            cv::putText(frame, "WAITING FOR CAMERA INFO...", cv::Point(30, 50),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 165, 255), 2);
            auto det_msg = std_msgs::msg::Bool();
            det_msg.data = false;
            pub_detected_->publish(det_msg);
            cv::imshow("Bebop Camera", frame);
            cv::waitKey(1);
            return;
        }
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        image_u8_t im = { gray.cols, gray.rows, gray.cols, gray.data };
        zarray_t* detections = apriltag_detector_detect(td_, &im);
        auto det_msg = std_msgs::msg::Bool();
        det_msg.data = (zarray_size(detections) > 0);
        pub_detected_->publish(det_msg);
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t* det;
            zarray_get(detections, i, &det);
            double s = tag_size_ / 2.0;
            std::vector<cv::Point3f> objPts = {{-s,-s,0}, {s,-s,0}, {s,s,0}, {-s,s,0}};
            std::vector<cv::Point2f> imgPts = {
                {(float)det->p[0][0], (float)det->p[0][1]},
                {(float)det->p[1][0], (float)det->p[1][1]},
                {(float)det->p[2][0], (float)det->p[2][1]},
                {(float)det->p[3][0], (float)det->p[3][1]}
            };
            cv::Mat rvec, tvec;
            cv::solvePnP(objPts, imgPts, cameraMatrix_, distCoeffs_, rvec, tvec);
            cv::Mat R_mat;
            cv::Rodrigues(rvec, R_mat);
            publish_tf(R_mat, tvec, det->id);
            draw_detection(frame, imgPts, det->id, rvec, tvec);
        }
        apriltag_detections_destroy(detections);
        cv::imshow("Bebop Camera", frame);
        if (cv::waitKey(1) == 'q') rclcpp::shutdown();
    }

    void publish_tf(const cv::Mat& R_mat, const cv::Mat& tvec, int id) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "camera_gimbal"; 
        t.child_frame_id = "tag_" + std::to_string(id);
        t.transform.translation.x = tvec.at<double>(0);
        t.transform.translation.y = tvec.at<double>(1);
        t.transform.translation.z = tvec.at<double>(2);
        tf2::Matrix3x3 tf2_rot(
            R_mat.at<double>(0,0), R_mat.at<double>(0,1), R_mat.at<double>(0,2),
            R_mat.at<double>(1,0), R_mat.at<double>(1,1), R_mat.at<double>(1,2),
            R_mat.at<double>(2,0), R_mat.at<double>(2,1), R_mat.at<double>(2,2)
        );
        tf2::Quaternion q;
        tf2_rot.getRotation(q);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(t);
    }

    void draw_detection(cv::Mat& frame, const std::vector<cv::Point2f>& pts, int id, const cv::Mat& rvec, const cv::Mat& tvec) {
        for (int i = 0; i < 4; i++)
            cv::line(frame, pts[i], pts[(i+1)%4], cv::Scalar(0, 255, 0), 2);
        cv::putText(frame, "ID: " + std::to_string(id), pts[0], 
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
        std::vector<cv::Point3f> axis_3d = {{0,0,0}, {0.1,0,0}, {0,0.1,0}, {0,0,0.1}};
        std::vector<cv::Point2f> axis_2d;
        cv::projectPoints(axis_3d, rvec, tvec, cameraMatrix_, distCoeffs_, axis_2d);
        cv::line(frame, axis_2d[0], axis_2d[1], cv::Scalar(255, 0, 0), 2);
        cv::line(frame, axis_2d[0], axis_2d[2], cv::Scalar(0, 255, 0), 2);
        cv::line(frame, axis_2d[0], axis_2d[3], cv::Scalar(0, 0, 255), 2);
    }

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_detected_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    apriltag_family_t* tf_;
    apriltag_detector_t* td_;
    bool has_camera_info_;
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;
    double tag_size_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BebopTagNode>());
    rclcpp::shutdown();
    return 0;
}