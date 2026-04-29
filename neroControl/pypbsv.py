#!/usr/bin/env python3
# Position-Based Visual Servoing (PBVS) control node with Alpha-Beta filtering for Bebop drone
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import math
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TransformStamped, Pose
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs
from tf_transformations import (
    quaternion_from_matrix, 
    quaternion_matrix, 
    euler_from_quaternion, 
    quaternion_multiply, 
    quaternion_from_euler
)
from pupil_apriltags import Detector

class BebopTagPBVS(Node):
    def __init__(self):
        super().__init__('bebop_tag_pbvs')
        self.detector = Detector(families='tag36h11')
        self.bridge = CvBridge()
        self.tag_size = 0.16
        self.has_camera_info = False
        self.camera_matrix = None
        self.dist_coeffs = None
        self.alpha = 0.85
        self.beta = 0.05
        self.dt = 1.0 / 30.0
        self.x_world_est = 0.0
        self.vx_world_est = 0.0
        self.y_world_est = 0.0
        self.vy_world_est = 0.0
        self.first_run = True
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.sub_info = self.create_subscription(CameraInfo, '/bebop/camera/camera_info', self.camera_info_callback, 10)
        self.sub_image = self.create_subscription(Image, '/bebop/camera/image_raw', self.image_callback, 10)
        self.pub_ref = self.create_publisher(Float64MultiArray, '/bebop/ref_vec', 10)

    def camera_info_callback(self, msg):
        if self.has_camera_info: return
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)
        self.has_camera_info = True

    def image_callback(self, msg):
        if not self.has_camera_info: return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception: return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray)
        if detections:
            det = detections[0]
            rvec, tvec = self.estimate_pose(det)
            pose_cam_frame = self.get_pose_from_pnp(rvec, tvec)
            self.publish_tag_tf(pose_cam_frame, msg.header.stamp, "camera_link", "tag_detected")
            try:
                tf_odom_cam = self.tf_buffer.lookup_transform("odom", "camera_link", rclpy.time.Time())
                pose_tag_odom = tf2_geometry_msgs.do_transform_pose(pose_cam_frame, tf_odom_cam)
                pose_ref = self.calculate_pose_ref(pose_tag_odom)
                z_x = pose_tag_odom.position.x
                z_y = pose_tag_odom.position.y
                if self.first_run:
                    self.x_world_est = z_x
                    self.y_world_est = z_y
                    self.first_run = False
                self.x_world_est += self.vx_world_est * self.dt
                self.y_world_est += self.vy_world_est * self.dt
                res_x = z_x - self.x_world_est
                res_y = z_y - self.y_world_est
                self.x_world_est += self.alpha * res_x
                self.y_world_est += self.alpha * res_y
                self.vx_world_est += (self.beta / self.dt) * res_x
                self.vy_world_est += (self.beta / self.dt) * res_y
                self.publish_reference(pose_ref, self.vx_world_est, self.vy_world_est)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass
        cv2.imshow("PBVS Monitor (Python)", frame)
        cv2.waitKey(1)

    def get_pose_from_pnp(self, rvec, tvec):
        pose = Pose()
        pose.position.x = float(tvec[2][0])
        pose.position.y = float(-tvec[0][0])
        pose.position.z = float(-tvec[1][0])
        rmat, _ = cv2.Rodrigues(rvec)
        t_mat = np.eye(4); t_mat[:3, :3] = rmat
        q_orig = quaternion_from_matrix(t_mat)
        q_rot = quaternion_from_euler(-math.pi/2, 0, -math.pi/2)
        q_final = quaternion_multiply(q_rot, q_orig)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q_final
        return pose

    def calculate_pose_ref(self, pose_tag_odom):
        q = [pose_tag_odom.orientation.x, pose_tag_odom.orientation.y, 
             pose_tag_odom.orientation.z, pose_tag_odom.orientation.w]
        mat_tag_global = quaternion_matrix(q)
        local_offset = np.array([-0.05, 0.0, 0.0, 1.0])
        target_offset_global = mat_tag_global @ local_offset
        pose_ref = Pose()
        pose_ref.position.x = pose_tag_odom.position.x + target_offset_global[0]
        pose_ref.position.y = pose_tag_odom.position.y + target_offset_global[1]
        pose_ref.position.z = 1.5
        pose_ref.orientation = pose_tag_odom.orientation
        return pose_ref

    def estimate_pose(self, det):
        s = self.tag_size / 2.0
        obj_p = np.array([[-s, s, 0.], [s, s, 0.], [s, -s, 0.], [-s, -s, 0.]], dtype=np.float32)
        img_p = np.array(det.corners, dtype=np.float32)
        _, rvec, tvec = cv2.solvePnP(obj_p, img_p, self.camera_matrix, self.dist_coeffs)
        return rvec, tvec

    def publish_tag_tf(self, pose, stamp, frame_id, child_id):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = frame_id
        t.child_frame_id = child_id
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def publish_reference(self, pose_ref, vx, vy):
        ref_msg = Float64MultiArray()
        q = [pose_ref.orientation.x, pose_ref.orientation.y, 
             pose_ref.orientation.z, pose_ref.orientation.w]
        _, _, yaw = euler_from_quaternion(q)
        vx_pub = round(vx, 3) if abs(vx) > 0.0001 else 0.0
        vy_pub = round(vy, 3) if abs(vy) > 0.0001 else 0.0
        ref_msg.data = [pose_ref.position.x, pose_ref.position.y, pose_ref.position.z, yaw, vx_pub, vy_pub, 0.0, 0.0]
        self.pub_ref.publish(ref_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BebopTagPBVS()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()