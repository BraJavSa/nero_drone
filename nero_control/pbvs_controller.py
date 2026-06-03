#!/usr/bin/env python3
"""
Bebop Drone — Position-Based Visual Servoing (PBVS) controller.
================================================================
Uses AprilTag pose estimation to generate a reference vector on
/bebop/ref_vec compatible with the velocity controller (context §6).

NumPy 2.x compatible:
  - No cv_bridge  (manual Image → ndarray conversion)
  - No tf_transformations (quaternion/rotation helpers implemented inline)

Reference published (Float64MultiArray, 12 elements):
    [0:4]  eta_d   = [x_ref, y_ref, z_ref, yaw_ref]   (pose setpoint)
    [4:8]  nu_d    = [vx, vy, 0, 0]                   (velocity feedforward, world frame)
    [8:12] alpha_d = [0, 0, 0, 0]                     (acceleration feedforward, unused here)

Alpha-beta tracker on x/y (world frame) provides the velocity feedforward.
"""

import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TransformStamped, Pose
import tf2_ros
import tf2_geometry_msgs

# NumPy-2-safe AprilTag import
try:
    from pupil_apriltags import Detector
except ImportError:
    from dt_apriltags import Detector


# ---------------------------------------------------------------------------
# Rotation / quaternion helpers (no tf_transformations dependency)
# ---------------------------------------------------------------------------

def _quat_from_matrix(m: np.ndarray) -> np.ndarray:
    """4x4 rotation matrix → quaternion [x, y, z, w]."""
    t = m[0, 0] + m[1, 1] + m[2, 2]
    if t > 0:
        s = 0.5 / math.sqrt(t + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w], dtype=np.float64)


def _matrix_from_quat(q: np.ndarray) -> np.ndarray:
    """Quaternion [x, y, z, w] → 4x4 homogeneous rotation matrix."""
    x, y, z, w = q
    m = np.eye(4, dtype=np.float64)
    m[0, 0] = 1 - 2*(y*y + z*z)
    m[0, 1] = 2*(x*y - z*w)
    m[0, 2] = 2*(x*z + y*w)
    m[1, 0] = 2*(x*y + z*w)
    m[1, 1] = 1 - 2*(x*x + z*z)
    m[1, 2] = 2*(y*z - x*w)
    m[2, 0] = 2*(x*z - y*w)
    m[2, 1] = 2*(y*z + x*w)
    m[2, 2] = 1 - 2*(x*x + y*y)
    return m


def _quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Quaternion product q1 * q2, both [x, y, z, w]."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ], dtype=np.float64)


def _quat_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Intrinsic XYZ Euler → quaternion [x, y, z, w]."""
    cr, sr = math.cos(roll/2),  math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2),   math.sin(yaw/2)
    return np.array([
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
        cr*cp*cy + sr*sp*sy,
    ], dtype=np.float64)


def _yaw_from_quat(q: np.ndarray) -> float:
    """Extract yaw from quaternion [x, y, z, w]."""
    x, y, z, w = q
    return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))


def _imgmsg_to_cv2(msg) -> np.ndarray:
    """Manual ROS Image → BGR ndarray, NumPy-2 safe, no cv_bridge."""
    n_ch = {'rgb8': 3, 'bgr8': 3, 'mono8': 1,
            'rgba8': 4, 'bgra8': 4}.get(msg.encoding, 3)
    img = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(
        msg.height, msg.width, n_ch)
    if msg.encoding == 'rgb8':
        img = img[:, :, ::-1]
    return np.ascontiguousarray(img)


# ---------------------------------------------------------------------------
# PBVS node
# ---------------------------------------------------------------------------

class BebopTagPBVS(Node):

    TAG_FAMILY       = 'tag36h11'
    TAG_SIZE         = 0.16          # metres
    TARGET_ALTITUDE  = 1.5           # metres above ground
    LOCAL_OFFSET     = np.array([-0.05, 0.0, 0.0, 1.0])  # hover slightly behind tag

    # Alpha-beta tracker gains (tune if velocity feedforward is too noisy)
    ALPHA = 0.85
    BETA  = 0.05
    DT    = 1.0 / 30.0              # image callback rate (approx.)

    def __init__(self):
        super().__init__('bebop_tag_pbvs')

        self.detector = Detector(
            families=self.TAG_FAMILY,
            nthreads=2,
            quad_decimate=2.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0,
        )

        # Camera
        self.camera_matrix   = None
        self.dist_coeffs     = None
        self.has_camera_info = False

        # Alpha-beta state (world frame x/y)
        self.x_est  = 0.0
        self.vx_est = 0.0
        self.y_est  = 0.0
        self.vy_est = 0.0
        self.first_run = True

        # TF2
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer      = tf2_ros.Buffer()
        self.tf_listener    = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriptions
        self.create_subscription(
            CameraInfo, '/bebop/camera/camera_info', self._camera_info_cb, 10)
        self.create_subscription(
            Image, '/bebop/camera/image_raw', self._image_cb, 10)

        # Publisher
        self.pub_ref = self.create_publisher(
            Float64MultiArray, '/bebop/ref_vec', 10)

        self.get_logger().info('BebopTagPBVS started (NumPy-2 compatible).')

    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------

    def _camera_info_cb(self, msg: CameraInfo):
        if self.has_camera_info:
            return
        self.camera_matrix   = np.array(msg.k, dtype=np.float64).reshape((3, 3))
        self.dist_coeffs     = np.array(msg.d, dtype=np.float64)
        self.has_camera_info = True

    def _image_cb(self, msg: Image):
        if not self.has_camera_info:
            return

        try:
            frame = _imgmsg_to_cv2(msg)
        except Exception:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        detections = self.detector.detect(gray)
        if not detections:
            cv2.imshow('PBVS Monitor', frame)
            cv2.waitKey(1)
            return

        det = detections[0]
        rvec, tvec = self._estimate_pose(det)
        pose_cam   = self._pose_from_pnp(rvec, tvec)

        # Broadcast tag TF for visualisation
        self._publish_tag_tf(pose_cam, msg.header.stamp, 'camera_link', 'tag_detected')

        # Transform tag pose to odom frame
        try:
            tf_odom_cam = self.tf_buffer.lookup_transform(
                'odom', 'camera_link', rclpy.time.Time())
            pose_odom = tf2_geometry_msgs.do_transform_pose(pose_cam, tf_odom_cam)
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            cv2.imshow('PBVS Monitor', frame)
            cv2.waitKey(1)
            return

        # Alpha-beta tracker for velocity feedforward
        z_x = pose_odom.position.x
        z_y = pose_odom.position.y

        if self.first_run:
            self.x_est  = z_x
            self.y_est  = z_y
            self.first_run = False

        self.x_est  += self.vx_est * self.DT
        self.y_est  += self.vy_est * self.DT
        res_x = z_x - self.x_est
        res_y = z_y - self.y_est
        self.x_est  += self.ALPHA * res_x
        self.y_est  += self.ALPHA * res_y
        self.vx_est += (self.BETA / self.DT) * res_x
        self.vy_est += (self.BETA / self.DT) * res_y

        # Build and publish reference
        pose_ref = self._calculate_pose_ref(pose_odom)
        self._publish_reference(pose_ref, self.vx_est, self.vy_est)

        cv2.imshow('PBVS Monitor', frame)
        cv2.waitKey(1)

    # -----------------------------------------------------------------------
    # Geometry helpers
    # -----------------------------------------------------------------------

    def _estimate_pose(self, det):
        """AprilTag corners → rvec, tvec via solvePnP."""
        s = self.TAG_SIZE / 2.0
        obj_pts = np.array([
            [-s,  s, 0.],
            [ s,  s, 0.],
            [ s, -s, 0.],
            [-s, -s, 0.],
        ], dtype=np.float32)
        img_pts = np.array(det.corners, dtype=np.float32)
        _, rvec, tvec = cv2.solvePnP(
            obj_pts, img_pts, self.camera_matrix, self.dist_coeffs)
        return rvec, tvec

    def _pose_from_pnp(self, rvec, tvec) -> Pose:
        """Convert OpenCV camera-frame pose → ROS Pose (camera optical convention)."""
        pose = Pose()
        # Axis remap: OpenCV Z-forward → ROS X-forward
        pose.position.x = float(tvec[2][0])
        pose.position.y = float(-tvec[0][0])
        pose.position.z = float(-tvec[1][0])

        rmat, _ = cv2.Rodrigues(rvec)
        t_mat = np.eye(4, dtype=np.float64)
        t_mat[:3, :3] = rmat

        q_orig = _quat_from_matrix(t_mat)
        q_rot  = _quat_from_euler(-math.pi/2, 0.0, -math.pi/2)
        q_fin  = _quat_multiply(q_rot, q_orig)

        pose.orientation.x = float(q_fin[0])
        pose.orientation.y = float(q_fin[1])
        pose.orientation.z = float(q_fin[2])
        pose.orientation.w = float(q_fin[3])
        return pose

    def _calculate_pose_ref(self, pose_odom: Pose) -> Pose:
        """Compute hover setpoint: slightly behind tag, at TARGET_ALTITUDE."""
        q = np.array([
            pose_odom.orientation.x,
            pose_odom.orientation.y,
            pose_odom.orientation.z,
            pose_odom.orientation.w,
        ], dtype=np.float64)
        mat = _matrix_from_quat(q)
        offset_world = mat @ self.LOCAL_OFFSET

        pose_ref = Pose()
        pose_ref.position.x   = pose_odom.position.x + offset_world[0]
        pose_ref.position.y   = pose_odom.position.y + offset_world[1]
        pose_ref.position.z   = self.TARGET_ALTITUDE
        pose_ref.orientation  = pose_odom.orientation
        return pose_ref

    # -----------------------------------------------------------------------
    # Publishers
    # -----------------------------------------------------------------------

    def _publish_tag_tf(self, pose: Pose, stamp, frame_id: str, child_id: str):
        t = TransformStamped()
        t.header.stamp    = stamp
        t.header.frame_id = frame_id
        t.child_frame_id  = child_id
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation      = pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def _publish_reference(self, pose_ref: Pose, vx: float, vy: float):
        """
        Publish reference vector per context §6 (12 elements):
            [0:4]  eta_d   — desired pose
            [4:8]  nu_d    — desired velocity (world frame)
            [8:12] alpha_d — desired acceleration (zero feedforward here)
        """
        q = np.array([
            pose_ref.orientation.x,
            pose_ref.orientation.y,
            pose_ref.orientation.z,
            pose_ref.orientation.w,
        ], dtype=np.float64)
        yaw = _yaw_from_quat(q)

        vx_pub = round(vx, 3) if abs(vx) > 1e-4 else 0.0
        vy_pub = round(vy, 3) if abs(vy) > 1e-4 else 0.0

        msg = Float64MultiArray()
        msg.data = [
            # eta_d
            float(pose_ref.position.x),
            float(pose_ref.position.y),
            float(pose_ref.position.z),
            float(yaw),
            # nu_d (world frame)
            vx_pub, vy_pub, 0.0, 0.0,
            # alpha_d
            0.0, 0.0, 0.0, 0.0,
        ]
        self.pub_ref.publish(msg)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

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