#!/usr/bin/env python3

# Image-Based Visual Servoing (IBVS) controller for vision-based target tracking.

import threading

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from pupil_apriltags import Detector
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64MultiArray


class PID:
    def __init__(self, kp: float, ki: float, kd: float,
                 output_limit: float = 1.0, deadband: float = 0.0,
                 integral_threshold: float = 0.15):
        self.kp                 = kp
        self.ki                 = ki
        self.kd                 = kd
        self.output_limit       = output_limit
        self.deadband           = deadband
        self.integral_threshold = integral_threshold
        self._integral          = 0.0
        self._prev_error        = 0.0
        self._prev_time         = None

    def reset(self):
        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_time  = None

    def compute(self, error: float, now: float, angular: bool = False) -> float:
        if abs(error) < self.deadband:
            error = 0.0

        if self._prev_time is None:
            self._prev_time  = now
            self._prev_error = error
            return float(np.clip(self.kp * error, -self.output_limit, self.output_limit))

        dt = now - self._prev_time
        if dt <= 1e-6:
            return float(np.clip(self.kp * error, -self.output_limit, self.output_limit))

        if abs(error) <= self.integral_threshold:
            self._integral += error * dt
        else:
            self._integral = 0.0

        integral_term = float(np.clip(self.ki * self._integral,
                                      -self.output_limit, self.output_limit))

        if angular:
            diff = (error - self._prev_error + np.pi) % (2.0 * np.pi) - np.pi
        else:
            diff = error - self._prev_error

        derivative = self.kd * diff / dt
        output     = float(np.clip(
            self.kp * error + integral_term + derivative,
            -self.output_limit, self.output_limit))

        self._prev_error = error
        self._prev_time  = now
        return output


class BebopTagNode(Node):

    TAG_FAMILY       = 'tag36h11'
    TAG_SIZE         = 0.165
    DESIRED_ALTITUDE = 0.80
    CONTROL_HZ       = 30.0

    PID_DEFAULTS = {
        'vx'  : dict(kp=0.3,  ki=0.05,  kd=0.06,      output_limit=1.0, deadband=0.02, integral_threshold=0.2),
        'vy'  : dict(kp=0.45, ki=0.08,  kd=0.0000005, output_limit=1.0, deadband=0.02, integral_threshold=0.2),
        'vz'  : dict(kp=0.50, ki=0.05,  kd=0.10,      output_limit=1.0, deadband=0.02, integral_threshold=0.1),
        'vyaw': dict(kp=1.0,  ki=0.01,  kd=0.001,     output_limit=0.8, deadband=0.02, integral_threshold=0.1),
    }

    def __init__(self):
        super().__init__('bebop_tag_node')

        self.detector = Detector(
            families=self.TAG_FAMILY,
            nthreads=2,
            quad_decimate=2.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0,
        )

        self.bridge          = CvBridge()
        self.camera_matrix   = None
        self.has_camera_info = False

        self._det_lock   = threading.Lock()
        self._latest     = None

        for axis, gains in self.PID_DEFAULTS.items():
            for name, val in gains.items():
                self.declare_parameter(f'{axis}.{name}', val)

        self.pid_vx   = self._pid_from_params('vx')
        self.pid_vy   = self._pid_from_params('vy')
        self.pid_vz   = self._pid_from_params('vz')
        self.pid_vyaw = self._pid_from_params('vyaw')

        self.add_on_set_parameters_callback(self._on_param_change)

        self.pub_ref = self.create_publisher(Float64MultiArray, '/bebop/ref_vec', 10)

        self.sub_info = self.create_subscription(
            CameraInfo, '/bebop/camera/camera_info', self.camera_info_cb, 10)
        self.sub_img  = self.create_subscription(
            Image, '/bebop/camera/image_raw', self.image_cb, 10)

        self.control_timer = self.create_timer(1.0 / self.CONTROL_HZ, self.control_loop_cb)

        self.get_logger().info(f'BebopTagNode started — control at {self.CONTROL_HZ} Hz')

    def _pid_from_params(self, axis: str) -> PID:
        g = lambda n: self.get_parameter(f'{axis}.{n}').value
        return PID(kp=g('kp'), ki=g('ki'), kd=g('kd'),
                   output_limit=g('output_limit'), deadband=g('deadband'),
                   integral_threshold=g('integral_threshold'))

    def _on_param_change(self, params):
        axes_changed = set()
        for p in params:
            parts = p.name.split('.')
            if len(parts) == 2 and parts[0] in self.PID_DEFAULTS:
                axes_changed.add(parts[0])
        for axis in axes_changed:
            pid = self._pid_from_params(axis)
            if   axis == 'vx'  : self.pid_vx   = pid
            elif axis == 'vy'  : self.pid_vy   = pid
            elif axis == 'vz'  : self.pid_vz   = pid
            elif axis == 'vyaw': self.pid_vyaw = pid
        return SetParametersResult(successful=True)

    def camera_info_cb(self, msg):
        if self.has_camera_info:
            return
        self.camera_matrix   = np.array(msg.k).reshape((3, 3))
        self.has_camera_info = True

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        return (angle + np.pi) % (2.0 * np.pi) - np.pi

    def image_cb(self, msg):
        if not self.has_camera_info:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return

        rows, cols = frame.shape[:2]
        gray       = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        detections = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=[
                self.camera_matrix[0, 0],
                self.camera_matrix[1, 1],
                self.camera_matrix[0, 2],
                self.camera_matrix[1, 2],
            ],
            tag_size=self.TAG_SIZE,
        )

        if detections:
            det  = detections[0]
            u, v = det.center
            pts  = det.corners
            alt  = float(det.pose_t.flatten()[2])

            e_u   = ((cols / 2.0) - u) / (cols / 2.0)
            e_v   = ((rows / 2.0) - v) / (rows / 2.0)
            e_z   = self.DESIRED_ALTITUDE - alt

            dx    = float(pts[1][0] - pts[0][0])
            dy    = float(pts[1][1] - pts[0][1])
            e_yaw = self._wrap_angle(np.arctan2(dy, dx) + np.pi / 2.0)

            new_det = dict(e_u=e_u, e_v=e_v, e_z=e_z, e_yaw=e_yaw)
        else:
            new_det = None

        with self._det_lock:
            self._latest = new_det

    def control_loop_cb(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        with self._det_lock:
            det = self._latest

        if det is not None:
            vx   =  self.pid_vx.compute(det['e_v'],   now)
            vy   =  self.pid_vy.compute(det['e_u'],   now)
            vz   =  self.pid_vz.compute(det['e_z'],   now)
            vyaw = -self.pid_vyaw.compute(det['e_yaw'], now, angular=True)
            self._publish(vx, vy, vz, vyaw)
        else:
            self.pid_vx.reset()
            self.pid_vy.reset()
            self.pid_vz.reset()
            self.pid_vyaw.reset()
            self._publish(0.0, 0.0, 0.0, 0.0)

    def _publish(self, vx, vy, vz, vyaw):
        msg      = Float64MultiArray()
        msg.data = [0.0] * 8
        msg.data[4] = float(vx)
        msg.data[5] = float(vy)
        msg.data[6] = float(vz)
        msg.data[7] = float(vyaw)
        self.pub_ref.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BebopTagNode()
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