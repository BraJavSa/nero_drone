#!/usr/bin/env python3
# Image-Based Visual Servoing (IBVS) node for Bebop drone using pupil_apriltags
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from pupil_apriltags import Detector

class BebopTagNode(Node):
    def __init__(self):
        super().__init__('bebop_tag_node')
        self.at_detector = Detector(
            families='tag36h11',
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.has_camera_info = False
        self.tag_size = 0.165
        self.pub_ref = self.create_publisher(Float64MultiArray, '/bebop/ref_vec', 10)
        self.sub_info = self.create_subscription(CameraInfo, '/bebop/camera/camera_info', self.camera_info_callback, 10)
        self.sub_image = self.create_subscription(Image, '/bebop/camera/image_raw', self.image_callback, 10)
        self.get_logger().info("BebopTagNode (Python) started.")

    def camera_info_callback(self, msg):
        if self.has_camera_info: return
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)
        self.has_camera_info = True

    def image_callback(self, msg):
        if not self.has_camera_info: return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self.at_detector.detect(
            gray, 
            estimate_tag_pose=True, 
            camera_params=[self.camera_matrix[0,0], self.camera_matrix[1,1], self.camera_matrix[0,2], self.camera_matrix[1,2]], 
            tag_size=self.tag_size
        )
        if len(detections) > 0:
            det = detections[0]
            pts = det.corners 
            u, v = det.center
            rows, cols = gray.shape
            e_u = (cols / 2.0) - u
            e_v = (rows / 2.0) - v
            dx = pts[2][0] - pts[1][0]
            dy = pts[2][1] - pts[1][1]
            e_yaw = 0.0 - np.arctan2(dx, -dy)
            e_z = 0.80 - det.pose_t[2] 
            self.publish_velocity(e_u, e_v, float(e_z), float(e_yaw))
            self.visualize(frame, pts, det.tag_id, u, v)
        else:
            self.publish_velocity(0.0, 0.0, 0.0, 0.0)
        cv2.imshow("Bebop IBVS Control (Python)", frame)
        cv2.waitKey(1)

    def publish_velocity(self, eu, ev, ez, eyaw):
        msg = Float64MultiArray()
        msg.data = [0.0] * 8
        kp_v = 0.001
        kp_z = 0.5
        kp_yaw = 0.5
        msg.data[4] = float(ev * kp_v)
        msg.data[5] = float(eu * kp_v)
        msg.data[6] = float(ez * kp_z)
        msg.data[7] = float(eyaw * kp_yaw)
        self.pub_ref.publish(msg)

    def visualize(self, frame, pts, tag_id, u, v):
        pts = pts.astype(int)
        for i in range(4):
            cv2.line(frame, tuple(pts[i]), tuple(pts[(i+1)%4]), (0, 255, 0), 2)
        cv2.circle(frame, (int(u), int(v)), 5, (0, 255, 255), -1)
        cv2.putText(frame, f"ID: {tag_id}", (int(pts[0][0]), int(pts[0][1]) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

def main(args=None):
    rclpy.init(args=args)
    node = BebopTagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()