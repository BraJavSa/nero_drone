// ROS2 node for AprilTag detection and 3D pose estimation using OpenCV and AprilTag 3
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <cmath>
#include <chrono>

class AprilTagNode : public rclcpp::Node {
public:
    AprilTagNode() : Node("apriltag_node") {
        cap_.open(0);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open camera");
            rclcpp::shutdown();
            return;
        }

        tf_ = tag36h11_create();
        td_ = apriltag_detector_create();
        apriltag_detector_add_family(td_, tf_);

        cameraMatrix_ = (cv::Mat1d(3,3) << 1296.0, 0, 679.1841,
                                           0, 1297.8, 352.7879,
                                           0, 0, 1);
        distCoeffs_ = (cv::Mat1d(1,5) << -0.1501, -0.1480, 0, 0, 0);

        tag_size_ = 0.12;

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&AprilTagNode::process_frame, this)
        );

        last_time_ = std::chrono::steady_clock::now();
        frame_count_ = 0;
        fps_ = 0.0;

        RCLCPP_INFO(this->get_logger(), "AprilTag node started with pose estimation.");
    }

    ~AprilTagNode() {
        apriltag_detector_destroy(td_);
        tag36h11_destroy(tf_);
    }

private:
    void process_frame() {
        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty()) return;

        frame_count_++;
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_time_).count();

        if (elapsed >= 1.0) {
            fps_ = frame_count_ / elapsed;
            RCLCPP_INFO(this->get_logger(), "FPS: %.2f", fps_);
            frame_count_ = 0;
            last_time_ = now;
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
                cv::Point3f(-s, -s, 0),
                cv::Point3f( s, -s, 0),
                cv::Point3f( s,  s, 0),
                cv::Point3f(-s,  s, 0)
            };

            std::vector<cv::Point2f> imagePoints = {
                {static_cast<float>(det->p[0][0]), static_cast<float>(det->p[0][1])},
                {static_cast<float>(det->p[1][0]), static_cast<float>(det->p[1][1])},
                {static_cast<float>(det->p[2][0]), static_cast<float>(det->p[2][1])},
                {static_cast<float>(det->p[3][0]), static_cast<float>(det->p[3][1])}
            };

            cv::Mat rvec, tvec;
            cv::solvePnP(objectPoints, imagePoints, cameraMatrix_, distCoeffs_, rvec, tvec);

            cv::Mat R_mat;
            cv::Rodrigues(rvec, R_mat);

            double sy = std::sqrt(R_mat.at<double>(0,0) * R_mat.at<double>(0,0) +
                                  R_mat.at<double>(1,0) * R_mat.at<double>(1,0));
            bool singular = sy < 1e-6;

            double roll, pitch, yaw;
            if (!singular) {
                roll  = std::atan2(R_mat.at<double>(2,1), R_mat.at<double>(2,2));
                pitch = std::atan2(-R_mat.at<double>(2,0), sy);
                yaw   = std::atan2(R_mat.at<double>(1,0), R_mat.at<double>(0,0));
            } else {
                roll  = std::atan2(-R_mat.at<double>(1,2), R_mat.at<double>(1,1));
                pitch = std::atan2(-R_mat.at<double>(2,0), sy);
                yaw   = 0;
            }

            double dist = cv::norm(tvec);
            RCLCPP_INFO(this->get_logger(),
                        "Tag %d: Pos [%.3f, %.3f, %.3f] m, Dist=%.2f m, Euler [R=%.2f, P=%.2f, Y=%.2f deg]",
                        det->id,
                        tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2),
                        dist,
                        roll*180/M_PI, pitch*180/M_PI, yaw*180/M_PI);

            for (int j = 0; j < 4; j++) {
                cv::line(frame, imagePoints[j], imagePoints[(j+1)%4], {0,255,0}, 2);
            }
            cv::putText(frame, std::to_string(det->id),
                        imagePoints[0], cv::FONT_HERSHEY_SIMPLEX, 0.7, {0,0,255}, 2);

            std::vector<cv::Point3f> axisPoints = {
                {0,0,0}, {0.1,0,0}, {0,0.1,0}, {0,0,0.1}
            };
            std::vector<cv::Point2f> imgpts;
            cv::projectPoints(axisPoints, rvec, tvec, cameraMatrix_, distCoeffs_, imgpts);

            cv::line(frame, imgpts[0], imgpts[1], {255,0,0}, 3);
            cv::line(frame, imgpts[0], imgpts[2], {0,255,0}, 3);
            cv::line(frame, imgpts[0], imgpts[3], {0,0,255}, 3);
        }

        apriltag_detections_destroy(detections);

        cv::putText(frame, "FPS: " + std::to_string(fps_), {10,30},
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, {255,255,0}, 2);

        cv::imshow("Camera", frame);
        if (cv::waitKey(1) == 'q') rclcpp::shutdown();
    }

    cv::VideoCapture cap_;
    apriltag_family_t *tf_;
    apriltag_detector_t *td_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;
    double tag_size_;
    std::chrono::steady_clock::time_point last_time_;
    int frame_count_;
    double fps_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AprilTagNode>());
    rclcpp::shutdown();
    return 0;
}
