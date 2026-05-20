// Handles video streaming integration and tag detection specifically tailored for the Bebop.



#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <cmath>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

class BebopTagNode : public rclcpp::Node {
public:
  BebopTagNode() : Node("bebop_tag_node") {
    tf_ = tag36h11_create();
    td_ = apriltag_detector_create();
    apriltag_detector_add_family(td_, tf_);

    sub_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/bebop/camera/camera_info", 10,
        std::bind(&BebopTagNode::camera_info_callback, this,
                  std::placeholders::_1));

    sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/bebop/camera/image_raw", 10,
        std::bind(&BebopTagNode::image_callback, this, std::placeholders::_1));

    tag_size_ = 0.12;
    RCLCPP_INFO(this->get_logger(), "Bebop AprilTag node started.");
  }

  ~BebopTagNode() {
    apriltag_detector_destroy(td_);
    tag36h11_destroy(tf_);
  }

private:
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (has_camera_info_)
      return;
    RCLCPP_INFO(this->get_logger(), "Camera info received.");
    cameraMatrix_ =
        (cv::Mat1d(3, 3) << msg->k[0], msg->k[1], msg->k[2], msg->k[3],
         msg->k[4], msg->k[5], msg->k[6], msg->k[7], msg->k[8]);
    distCoeffs_ = cv::Mat(msg->d).clone();
    has_camera_info_ = true;
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!has_camera_info_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Waiting for camera info...");
      return;
    }

    cv::Mat frame;
    try {
      frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    image_u8_t im = {gray.cols, gray.rows, gray.cols, gray.data};
    zarray_t *detections = apriltag_detector_detect(td_, &im);

    for (int i = 0; i < zarray_size(detections); i++) {
      apriltag_detection_t *det;
      zarray_get(detections, i, &det);

      double s = tag_size_ / 2.0;
      std::vector<cv::Point3f> objectPoints = {
          {-s, -s, 0}, {s, -s, 0}, {s, s, 0}, {-s, s, 0}};
      std::vector<cv::Point2f> imagePoints = {
          {static_cast<float>(det->p[0][0]), static_cast<float>(det->p[0][1])},
          {static_cast<float>(det->p[1][0]), static_cast<float>(det->p[1][1])},
          {static_cast<float>(det->p[2][0]), static_cast<float>(det->p[2][1])},
          {static_cast<float>(det->p[3][0]), static_cast<float>(det->p[3][1])}};

      cv::Mat rvec, tvec;
      cv::solvePnP(objectPoints, imagePoints, cameraMatrix_, distCoeffs_, rvec,
                   tvec);

      cv::Mat R_mat;
      cv::Rodrigues(rvec, R_mat);

      double sy = std::sqrt(R_mat.at<double>(0, 0) * R_mat.at<double>(0, 0) +
                            R_mat.at<double>(1, 0) * R_mat.at<double>(1, 0));
      bool singular = sy < 1e-6;

      double roll, pitch, yaw;
      if (!singular) {
        roll = std::atan2(R_mat.at<double>(2, 1), R_mat.at<double>(2, 2));
        pitch = std::atan2(-R_mat.at<double>(2, 0), sy);
        yaw = std::atan2(R_mat.at<double>(1, 0), R_mat.at<double>(0, 0));
      } else {
        roll = std::atan2(-R_mat.at<double>(1, 2), R_mat.at<double>(1, 1));
        pitch = std::atan2(-R_mat.at<double>(2, 0), sy);
        yaw = 0;
      }

      double dist = cv::norm(tvec);

      RCLCPP_INFO(this->get_logger(),
                  "Tag %d: Pos [%.3f, %.3f, %.3f] m, Dist=%.2f m, Euler "
                  "[R=%.2f, P=%.2f, Y=%.2f deg]",
                  det->id, tvec.at<double>(0), tvec.at<double>(1),
                  tvec.at<double>(2), dist, roll * 180 / M_PI,
                  pitch * 180 / M_PI, yaw * 180 / M_PI);

      for (int j = 0; j < 4; j++) {
        cv::line(frame, imagePoints[j], imagePoints[(j + 1) % 4], {0, 255, 0},
                 2);
      }
      cv::putText(frame, std::to_string(det->id), imagePoints[0],
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 0, 255}, 2);

      std::vector<cv::Point3f> axisPoints = {
          {0, 0, 0}, {0.1, 0, 0}, {0, 0.1, 0}, {0, 0, 0.1}};
      std::vector<cv::Point2f> imgpts;
      cv::projectPoints(axisPoints, rvec, tvec, cameraMatrix_, distCoeffs_,
                        imgpts);

      cv::line(frame, imgpts[0], imgpts[1], {255, 0, 0}, 3);
      cv::line(frame, imgpts[0], imgpts[2], {0, 255, 0}, 3);
      cv::line(frame, imgpts[0], imgpts[3], {0, 0, 255}, 3);
    }

    apriltag_detections_destroy(detections);
    cv::imshow("Bebop Camera", frame);
    if (cv::waitKey(1) == 'q')
      rclcpp::shutdown();
  }

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  apriltag_family_t *tf_;
  apriltag_detector_t *td_;
  cv::Mat cameraMatrix_;
  cv::Mat distCoeffs_;
  bool has_camera_info_ = false;
  double tag_size_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BebopTagNode>());
  rclcpp::shutdown();
  return 0;
}
