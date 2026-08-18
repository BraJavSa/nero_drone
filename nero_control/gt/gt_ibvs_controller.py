#!/usr/bin/env python3
"""
Ground Truth IBVS Controller for Bebop drone.
Subscribes to /bebop/gt_fullodom and camera info to perform image-based visual servoing.
"""

import threading
import math
import cv2
import numpy as np
import rclpy
import tf2_ros
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64MultiArray

try:
    from pupil_apriltags import Detector
except ImportError:
    from dt_apriltags import Detector


class GtBebopTagNode(Node):

    TAG_FAMILY       = 'tag36h11'
    TAG_SIZE         = 0.30
    DESIRED_ALTITUDE = 1.5
    CONTROL_HZ       = 15.0

    GAIN_DEFAULTS = {
        'lambda_vx': 2.0,
        'lambda_vy': 0.7,
        'lambda_vz': 0.68,
        'lambda_yaw': 1.32,
        'tau_out': 0.2,
    }
    OUTPUT_LIMIT_LINEAR  = 1.0
    OUTPUT_LIMIT_ANGULAR = 1.0

    def __init__(self):
        super().__init__('gt_bebop_tag_node')

        self.detector = Detector(
            families=self.TAG_FAMILY,
            nthreads=2,
            quad_decimate=2.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0,
        )

        self.camera_matrix   = None
        self.has_camera_info = False

        self._det_lock = threading.Lock()
        self._img_lock = threading.Lock()
        self._latest    = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_vz = 0.0
        self.cmd_vyaw = 0.0

        self.gimbal_pitch_current = -10.0
        self.gimbal_pitch_target  = -10.0
        self.gimbal_waiting       = False
        self.gimbal_moving        = False
        self.gimbal_wait_start   = 0.0
        self.gimbal_delay         = 0.5
        self.gimbal_speed         = 22.0
        self.last_time            = None

        for name, val in self.GAIN_DEFAULTS.items():
            self.declare_parameter(name, val)

        self.add_on_set_parameters_callback(self._on_param_change)

        self.pub_ref = self.create_publisher(Float64MultiArray, '/bebop/ref_vec', 10)

        self.sub_info = self.create_subscription(
            CameraInfo, '/bebop/camera/camera_info', self.camera_info_cb, 10)
        self.sub_img  = self.create_subscription(
            Image, '/bebop/camera/image_raw', self.image_cb, 1)
        self.sub_move_camera = self.create_subscription(
            Vector3, '/bebop/move_camera', self.move_camera_cb, 10)

        self.control_timer = self.create_timer(1.0 / self.CONTROL_HZ, self.control_loop_cb)

        self.get_logger().info(f'GtBebopTagNode started — control at {self.CONTROL_HZ} Hz')

    def _on_param_change(self, params):
        return SetParametersResult(successful=True)

    def camera_info_cb(self, msg):
        if self.has_camera_info:
            return
        self.camera_matrix   = np.array(msg.k, dtype=np.float64).reshape((3, 3))
        self.has_camera_info = True

    def move_camera_cb(self, msg: Vector3):
        target = float(np.clip(msg.x, -90.0, 15.0))
        if abs(target - self.gimbal_pitch_target) < 0.5:
            return
        self.gimbal_pitch_target = target
        self.gimbal_moving       = True
        self.gimbal_waiting      = True
        self.gimbal_wait_start   = self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _imgmsg_to_cv2(msg) -> np.ndarray:
        n_ch = {'rgb8': 3, 'bgr8': 3, 'mono8': 1,
                'rgba8': 4, 'bgra8': 4}.get(msg.encoding, 3)
        img = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(
            msg.height, msg.width, n_ch)
        if msg.encoding == 'rgb8':
            img = img[:, :, ::-1]
        return np.ascontiguousarray(img)

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        return (angle + np.pi) % (2.0 * np.pi) - np.pi

    @staticmethod
    def _quat_to_rot_matrix(qx, qy, qz, qw):
        R = np.array([
            [1.0 - 2.0*(qy**2 + qz**2), 2.0*(qx*qy - qz*qw), 2.0*(qx*qz + qy*qw)],
            [2.0*(qx*qy + qz*qw), 1.0 - 2.0*(qx**2 + qz**2), 2.0*(qy*qz - qx*qw)],
            [2.0*(qx*qz - qy*qw), 2.0*(qy*qz + qx*qw), 1.0 - 2.0*(qx**2 + qy**2)]
        ], dtype=np.float64)
        return R

    def image_cb(self, msg):
        if not self.has_camera_info:
            return

        if not self._img_lock.acquire(blocking=False):
            return

        try:
            img = self._imgmsg_to_cv2(msg)
            if len(img.shape) == 3:
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            else:
                gray = img

            fx, fy = float(self.camera_matrix[0, 0]), float(self.camera_matrix[1, 1])
            cx, cy = float(self.camera_matrix[0, 2]), float(self.camera_matrix[1, 2])

            detections = self.detector.detect(
                gray,
                estimate_tag_pose=False,
                camera_params=[fx, fy, cx, cy],
                tag_size=self.TAG_SIZE,
            )

            if detections:
                det  = detections[0]
                u, v = det.center
                pts  = det.corners

                diag_px = 0.5 * (
                    np.linalg.norm(pts[2] - pts[0]) +
                    np.linalg.norm(pts[3] - pts[1])
                )
                tag_diag_m = self.TAG_SIZE * np.sqrt(2.0)
                f_avg = 0.5 * (fx + fy)
                Z = float(f_avg * tag_diag_m / diag_px) if diag_px > 1e-6 else self.DESIRED_ALTITUDE

                x = (u - cx) / fx
                y = (v - cy) / fy

                dx = float(pts[1][0] - pts[0][0])
                dy = float(pts[1][1] - pts[0][1])
                yaw_feat = self._wrap_angle(np.arctan2(dy, dx) + np.pi / 2.0)

                new_det = dict(x=x, y=y, Z=Z, yaw=yaw_feat, stamp=self.get_clock().now().nanoseconds * 1e-9)
                with self._det_lock:
                    self._latest = new_det
        except Exception:
            pass
        finally:
            self._img_lock.release()

    def _compute_rotation_and_jacobian(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        alpha = -np.radians(self.gimbal_pitch_current)
        sin_a = np.sin(alpha)
        cos_a = np.cos(alpha)
        
        R_b_c = np.array([
            [0.0,    -1.0, 0.0],
            [-sin_a, 0.0,  -cos_a],
            [cos_a,  0.0,  -sin_a]
        ], dtype=np.float64)
        
        xc = 0.12
        
        J_v = np.array([
            [0.0,    -1.0, 0.0,    -xc],
            [-sin_a, 0.0,  -cos_a, 0.0],
            [cos_a,  0.0,  -sin_a, 0.0]
        ], dtype=np.float64)
        
        J_w = np.array([
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, -cos_a],
            [0.0, 0.0, 0.0, -sin_a]
        ], dtype=np.float64)
        
        return R_b_c, J_v, J_w

    def control_loop_cb(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_time is not None:
            dt = now - self.last_time
            if self.gimbal_waiting:
                if (now - self.gimbal_wait_start) >= self.gimbal_delay:
                    self.gimbal_waiting = False
            if self.gimbal_moving and not self.gimbal_waiting:
                speed = self.gimbal_speed * dt
                error = self.gimbal_pitch_target - self.gimbal_pitch_current
                if abs(error) <= speed:
                    self.gimbal_pitch_current = self.gimbal_pitch_target
                    self.gimbal_moving        = False
                else:
                    self.gimbal_pitch_current += np.sign(error) * speed
        self.last_time = now

        with self._det_lock:
            det = self._latest

        if det is None or (now - det['stamp']) > 0.4:
            self._publish(0.0, 0.0, 0.0, 0.0)
            return

        lam_vx   = self.get_parameter('lambda_vx').value
        lam_vy   = self.get_parameter('lambda_vy').value
        lam_vz   = self.get_parameter('lambda_vz').value
        lam_yaw  = self.get_parameter('lambda_yaw').value

        s_star = np.array([0.0, 0.0, self.DESIRED_ALTITUDE, 0.0])
        s      = np.array([det['x'], det['y'], det['Z'], det['yaw']])
        e      = s - s_star

        x, y, Z = det['x'], det['y'], det['Z']
        
        L_linear = np.array([
            [-1.0/Z, 0.0,    x/Z],
            [0.0,    -1.0/Z, y/Z],
            [0.0,    0.0,    -1.0],
            [0.0,    0.0,    0.0]
        ], dtype=np.float64)
        
        L_angular = np.array([
            [x*y,       -(1.0 + x**2),  y],
            [1.0 + y**2, -x*y,          -x],
            [-y*Z,       x*Z,           0.0],
            [0.0,        0.0,           -1.0]
        ], dtype=np.float64)

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                "camera_gimbal",
                "bebop_gt",
                rclpy.time.Time()
            )
            xc = tf_msg.transform.translation.x
            yc = tf_msg.transform.translation.y
            zc = tf_msg.transform.translation.z
            
            qx = tf_msg.transform.rotation.x
            qy = tf_msg.transform.rotation.y
            qz = tf_msg.transform.rotation.z
            qw = tf_msg.transform.rotation.w
            
            R_b_gimbal = self._quat_to_rot_matrix(qx, qy, qz, qw)
            
            R_g_opt = np.array([
                [0.0, -1.0,  0.0],
                [0.0,  0.0, -1.0],
                [1.0,  0.0,  0.0]
            ], dtype=np.float64)
            
            R_b_c = R_g_opt @ R_b_gimbal
            
            J_v = np.zeros((3, 4), dtype=np.float64)
            J_v[:, 0:3] = R_b_c
            J_v[:, 3] = R_b_c @ np.array([-yc, xc, 0.0])
            
            J_w = np.zeros((3, 4), dtype=np.float64)
            J_w[:, 3] = R_b_c[:, 2]
            
        except Exception as err:
            _, J_v, J_w = self._compute_rotation_and_jacobian()

        L_body = L_linear @ J_v + L_angular @ J_w

        lam_vx_val  = lam_vx  * (1.0 + np.exp(-10.0 * abs(e[0])))
        lam_vy_val  = lam_vy  * (1.0 + np.exp(-10.0 * abs(e[1])))
        lam_vz_val  = lam_vz  * (1.0 + np.exp(-5.0  * abs(e[2])))
        lam_yaw_val = lam_yaw * (1.0 + np.exp(-5.0  * abs(e[3])))

        z_scale = np.clip(self.DESIRED_ALTITUDE / Z, 0.3, 3.0)
        Lam = np.diag([lam_vx_val, lam_vy_val, lam_vz_val, lam_yaw_val]) * z_scale

        L_body_pinv = np.linalg.pinv(L_body)
        v_b = -Lam @ (L_body_pinv @ e)

        vx, vy, vz, vyaw = v_b

        vx = float(np.clip(vx, -self.OUTPUT_LIMIT_LINEAR, self.OUTPUT_LIMIT_LINEAR))
        vy = float(np.clip(vy, -self.OUTPUT_LIMIT_LINEAR, self.OUTPUT_LIMIT_LINEAR))
        vz = float(np.clip(vz, -self.OUTPUT_LIMIT_LINEAR, self.OUTPUT_LIMIT_LINEAR))
        vyaw = float(np.clip(vyaw, -self.OUTPUT_LIMIT_ANGULAR, self.OUTPUT_LIMIT_ANGULAR))

        self._publish(vx, vy, vz, vyaw)

    def _publish(self, vx, vy, vz, vyaw):
        tau_out = self.get_parameter('tau_out').value
        alpha_out = (1.0 / self.CONTROL_HZ) / (tau_out + (1.0 / self.CONTROL_HZ))
        
        self.cmd_vx += alpha_out * (vx - self.cmd_vx)
        self.cmd_vy += alpha_out * (vy - self.cmd_vy)
        self.cmd_vz += alpha_out * (vz - self.cmd_vz)
        self.cmd_vyaw += alpha_out * (vyaw - self.cmd_vyaw)

        msg      = Float64MultiArray()
        msg.data = [
            0.0, 0.0, 0.0, 0.0,
            float(self.cmd_vx),
            float(self.cmd_vy),
            float(self.cmd_vz),
            float(self.cmd_vyaw),
            0.0, 0.0, 0.0, 0.0
        ]
        self.pub_ref.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GtBebopTagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
