// ROS2 node for tracking AprilTags in world frame (odom) using an alpha-beta filter
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <unordered_map>
#include <unordered_set>
#include <cmath>

class BebopTagWorldTrackerNode : public rclcpp::Node {
public:
    BebopTagWorldTrackerNode()
    : Node("bebop_tag_world_tracker_node")
    {
        alpha_ = this->declare_parameter<double>("alpha", 0.7);
        beta_  = this->declare_parameter<double>("beta", 0.2);
        tag_size_ = this->declare_parameter<double>("tag_size", 0.12);
        vel_lambda_ = this->declare_parameter<double>("vel_lambda", 1.0);
        max_missing_frames_ = this->declare_parameter<int>("max_missing_frames", 10);
        world_frame_  = this->declare_parameter<std::string>("world_frame",  "odom");
        camera_frame_ = this->declare_parameter<std::string>("camera_frame", "camera_link");

        tag_family_ = tag36h11_create();
        tag_detector_ = apriltag_detector_create();
        apriltag_detector_add_family(tag_detector_, tag_family_);

        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        sub_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/bebop/camera/camera_info", 10,
            std::bind(&BebopTagWorldTrackerNode::camera_info_callback, this, std::placeholders::_1));

        sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/bebop/camera/image_raw", 10,
            std::bind(&BebopTagWorldTrackerNode::image_callback, this, std::placeholders::_1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("tag_odom", 10);

        RCLCPP_INFO(this->get_logger(),
                    "BebopTagWorldTrackerNode started. alpha=%.3f, beta=%.3f, tag_size=%.3f m, "
                    "vel_lambda=%.3f, max_missing_frames=%d, world_frame=%s, camera_frame=%s",
                    alpha_, beta_, tag_size_, vel_lambda_, max_missing_frames_,
                    world_frame_.c_str(), camera_frame_.c_str());
    }

    ~BebopTagWorldTrackerNode() override
    {
        apriltag_detector_destroy(tag_detector_);
        tag36h11_destroy(tag_family_);
    }

private:
    struct AlphaBetaState {
        cv::Vec3d x;
        cv::Vec3d v;
        geometry_msgs::msg::Quaternion orientation;
        rclcpp::Time last_time;
        int frames_missing = 0;
        bool initialized = false;
        bool valid = false;
    };

    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (has_camera_info_) return;
        RCLCPP_INFO(this->get_logger(), "Camera info received.");
        camera_matrix_ = (cv::Mat1d(3,3) <<
            msg->k[0], msg->k[1], msg->k[2],
            msg->k[3], msg->k[4], msg->k[5],
            msg->k[6], msg->k[7], msg->k[8]);
        dist_coeffs_ = cv::Mat(msg->d).clone();
        has_camera_info_ = true;
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!has_camera_info_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Waiting for camera info...");
            return;
        }

        cv::Mat frame;
        try { frame = cv_bridge::toCvCopy(msg, "bgr8")->image; }
        catch (cv_bridge::Exception& e) { return; }

        rclcpp::Time stamp(msg->header.stamp);
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        image_u8_t im = { gray.cols, gray.rows, gray.cols, gray.data };
        zarray_t* detections = apriltag_detector_detect(tag_detector_, &im);

        std::unordered_set<int> seen_ids;
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t* det;
            zarray_get(detections, i, &det);
            int id = det->id;
            seen_ids.insert(id);

            double s = tag_size_ / 2.0;
            std::vector<cv::Point3f> object_points = {
                {-s, -s, 0}, { s, -s, 0}, { s,  s, 0}, { -s,  s, 0}
            };
            std::vector<cv::Point2f> image_points = {
                {float(det->p[0][0]), float(det->p[0][1])},
                {float(det->p[1][0]), float(det->p[1][1])},
                {float(det->p[2][0]), float(det->p[2][1])},
                {float(det->p[3][0]), float(det->p[3][1])}
            };

            cv::Mat rvec, tvec;
            cv::solvePnP(object_points, image_points, camera_matrix_, dist_coeffs_, rvec, tvec);
            cv::Mat R_mat;
            cv::Rodrigues(rvec, R_mat);
            cv::Vec3d pos_cam(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

            tf2::Matrix3x3 m_ct(R_mat.at<double>(0,0), R_mat.at<double>(0,1), R_mat.at<double>(0,2),
                              R_mat.at<double>(1,0), R_mat.at<double>(1,1), R_mat.at<double>(1,2),
                              R_mat.at<double>(2,0), R_mat.at<double>(2,1), R_mat.at<double>(2,2));
            tf2::Quaternion q_ct;
            m_ct.getRotation(q_ct);

            geometry_msgs::msg::PoseStamped pose_cam;
            pose_cam.header.stamp = stamp;
            pose_cam.header.frame_id = camera_frame_;
            pose_cam.pose.position.x = pos_cam[0];
            pose_cam.pose.position.y = pos_cam[1];
            pose_cam.pose.position.z = pos_cam[2];
            pose_cam.pose.orientation = tf2::toMsg(q_ct);

            geometry_msgs::msg::PoseStamped pose_world;
            try {
                geometry_msgs::msg::TransformStamped tf_cam_to_world = tf_buffer_->lookupTransform(world_frame_, camera_frame_, stamp);
                tf2::doTransform(pose_cam, pose_world, tf_cam_to_world);
            } catch (const tf2::TransformException &ex) { continue; }

            cv::Vec3d pos_world(pose_world.pose.position.x, pose_world.pose.position.y, pose_world.pose.position.z);
            auto &state = states_[id];
            if (!state.initialized) {
                state.x = pos_world;
                state.v = cv::Vec3d(0.0, 0.0, 0.0);
                state.orientation = pose_world.pose.orientation;
                state.last_time = stamp;
                state.frames_missing = 0;
                state.initialized = true;
                state.valid = true;
            } else {
                double dt = (stamp - state.last_time).seconds();
                if (dt > 0.0) {
                    cv::Vec3d x_pred = state.x + state.v * dt;
                    cv::Vec3d v_pred = state.v;
                    cv::Vec3d r = pos_world - x_pred;
                    state.x = x_pred + alpha_ * r;
                    state.v = v_pred + (beta_ / dt) * r;
                    state.orientation = pose_world.pose.orientation;
                } else {
                    state.x = pos_world;
                    state.orientation = pose_world.pose.orientation;
                }
                state.last_time = stamp;
                state.frames_missing = 0;
                state.valid = true;
            }

            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header.stamp = stamp;
            odom_msg.header.frame_id = world_frame_;
            odom_msg.child_frame_id = "tag_" + std::to_string(id);
            odom_msg.pose.pose.position.x = state.x[0];
            odom_msg.pose.pose.position.y = state.x[1];
            odom_msg.pose.pose.position.z = state.x[2];
            odom_msg.pose.pose.orientation = state.orientation;
            odom_msg.twist.twist.linear.x = state.v[0];
            odom_msg.twist.twist.linear.y = state.v[1];
            odom_msg.twist.twist.linear.z = state.v[2];
            odom_pub_->publish(odom_msg);

            for (int j = 0; j < 4; j++)
                cv::line(frame, image_points[j], image_points[(j+1)%4], cv::Scalar(0, 255, 0), 2);
            cv::putText(frame, std::to_string(id), image_points[0], cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        }

        for (auto &kv : states_) {
            int id = kv.first;
            auto &state = kv.second;
            if (!state.initialized || seen_ids.find(id) != seen_ids.end()) continue;
            double dt = (stamp - state.last_time).seconds();
            if (dt <= 0.0) continue;
            state.x = state.x + state.v * dt;
            state.v *= std::exp(-vel_lambda_ * dt);
            state.last_time = stamp;
            state.frames_missing++;
            if (state.frames_missing > max_missing_frames_) state.valid = false;

            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header.stamp = stamp;
            odom_msg.header.frame_id = world_frame_;
            odom_msg.child_frame_id = "tag_" + std::to_string(id);
            odom_msg.pose.pose.position.x = state.x[0];
            odom_msg.pose.pose.position.y = state.x[1];
            odom_msg.pose.pose.position.z = state.x[2];
            odom_msg.pose.pose.orientation = state.orientation;
            odom_msg.twist.twist.linear.x = state.v[0];
            odom_msg.twist.twist.linear.y = state.v[1];
            odom_msg.twist.twist.linear.z = state.v[2];
            odom_pub_->publish(odom_msg);
        }
        apriltag_detections_destroy(detections);
        cv::imshow("Bebop Tag World Tracker", frame);
        if (cv::waitKey(1) == 'q') rclcpp::shutdown();
    }

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    apriltag_family_t *tag_family_ = nullptr;
    apriltag_detector_t *tag_detector_ = nullptr;
    cv::Mat camera_matrix_, dist_coeffs_;
    bool has_camera_info_ = false;
    double tag_size_, alpha_, beta_, vel_lambda_;
    int max_missing_frames_;
    std::string world_frame_, camera_frame_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unordered_map<int, AlphaBetaState> states_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BebopTagWorldTrackerNode>());
    rclcpp::shutdown();
    return 0;
}
