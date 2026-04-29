// ROS2 node for Position-Based Visual Servoing (PBVS) using AprilTags
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

class BebopTagPBVS : public rclcpp::Node {
public:
    BebopTagPBVS() : Node("bebop_tag_pbvs") {
        tf_ = tag36h11_create();
        td_ = apriltag_detector_create();
        apriltag_detector_add_family(td_, tf_);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        sub_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/bebop/camera/camera_info", 10,
            std::bind(&BebopTagPBVS::camera_info_callback, this, std::placeholders::_1));

        sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/bebop/camera/image_raw", 10,
            std::bind(&BebopTagPBVS::image_callback, this, std::placeholders::_1));

        pub_ref_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/bebop/ref_vec", 10);

        tag_size_ = 0.30; 
    }

    ~BebopTagPBVS() {
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

        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t* det;
            zarray_get(detections, i, &det);

            cv::Mat rvec, tvec;
            estimate_pose(det, rvec, tvec);

            double tx =  tvec.at<double>(2);
            double ty = -tvec.at<double>(0);
            double tz = -tvec.at<double>(1);

            geometry_msgs::msg::Pose pose_cam_frame;
            pose_cam_frame.position.x = tx;
            pose_cam_frame.position.y = ty;
            pose_cam_frame.position.z = tz;

            cv::Mat R_mat;
            cv::Rodrigues(rvec, R_mat);
            tf2::Matrix3x3 tf2_rot(
                R_mat.at<double>(0,0), R_mat.at<double>(0,1), R_mat.at<double>(0,2),
                R_mat.at<double>(1,0), R_mat.at<double>(1,1), R_mat.at<double>(1,2),
                R_mat.at<double>(2,0), R_mat.at<double>(2,1), R_mat.at<double>(2,2));

            tf2::Quaternion q_orig;
            tf2_rot.getRotation(q_orig);

            tf2::Quaternion q_rot;
            q_rot.setRPY(-M_PI/2, 0, -M_PI/2); 
            tf2::Quaternion q_final = q_rot * q_orig;
            q_final.normalize();
            pose_cam_frame.orientation = tf2::toMsg(q_final);

            publish_tag_tf(pose_cam_frame, msg->header.stamp, "camera_link", "tag_detected");

            try {
                geometry_msgs::msg::TransformStamped tf_odom_cam = tf_buffer_->lookupTransform(
                    "odom", "camera_link", tf2::TimePointZero);

                geometry_msgs::msg::Pose pose_tag_odom;
                tf2::doTransform(pose_cam_frame, pose_tag_odom, tf_odom_cam);

                tf2::Transform tf_tag_global;
                tf2::fromMsg(pose_tag_odom, tf_tag_global);

                tf2::Vector3 local_offset(-0.05, 0.0, 0.0); 
                tf2::Vector3 target_pos_odom = tf_tag_global * local_offset;

                geometry_msgs::msg::Pose pose_ref;
                pose_ref.position.x = target_pos_odom.x();
                pose_ref.position.y = target_pos_odom.y();
                pose_ref.position.z = 1.5; 
                pose_ref.orientation = pose_tag_odom.orientation;

                publish_reference(pose_ref);

            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
            }
        }
        apriltag_detections_destroy(detections);
        cv::imshow("PBVS Monitor", frame);
        cv::waitKey(1);
    }

    void estimate_pose(apriltag_detection_t* det, cv::Mat& rvec, cv::Mat& tvec) {
        float s = static_cast<float>(tag_size_ / 2.0);
        std::vector<cv::Point3f> objP = {
            cv::Point3f(-s,  s, 0.0f), cv::Point3f( s,  s, 0.0f), 
            cv::Point3f( s, -s, 0.0f), cv::Point3f(-s, -s, 0.0f)
        };
        std::vector<cv::Point2f> imgP = {
            {static_cast<float>(det->p[0][0]), static_cast<float>(det->p[0][1])},
            {static_cast<float>(det->p[1][0]), static_cast<float>(det->p[1][1])},
            {static_cast<float>(det->p[2][0]), static_cast<float>(det->p[2][1])},
            {static_cast<float>(det->p[3][0]), static_cast<float>(det->p[3][1])}
        };
        cv::solvePnP(objP, imgP, cameraMatrix_, distCoeffs_, rvec, tvec);
    }

    void publish_tag_tf(const geometry_msgs::msg::Pose& pose, rclcpp::Time stamp, std::string frame_id, std::string child_id) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp;
        t.header.frame_id = frame_id;
        t.child_frame_id = child_id;
        t.transform.translation.x = pose.position.x;
        t.transform.translation.y = pose.position.y;
        t.transform.translation.z = pose.position.z;
        t.transform.rotation = pose.orientation;
        tf_broadcaster_->sendTransform(t);
    }

    void publish_reference(const geometry_msgs::msg::Pose& pose_ref) {
        auto ref_msg = std_msgs::msg::Float64MultiArray();
        tf2::Quaternion q;
        tf2::fromMsg(pose_ref.orientation, q);
        double r, p, yaw;
        tf2::Matrix3x3(q).getRPY(r, p, yaw);

        ref_msg.data.resize(8);
        ref_msg.data[0] = pose_ref.position.x;
        ref_msg.data[1] = pose_ref.position.y;
        ref_msg.data[2] = pose_ref.position.z;
        ref_msg.data[3] = yaw;
        for(int i=4; i<8; i++) ref_msg.data[i] = 0.0;
        pub_ref_->publish(ref_msg);
    }

    apriltag_family_t *tf_;
    apriltag_detector_t *td_;
    cv::Mat cameraMatrix_, distCoeffs_;
    bool has_camera_info_ = false;
    double tag_size_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_ref_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BebopTagPBVS>());
    rclcpp::shutdown();
    return 0;
}