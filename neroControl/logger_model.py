#!/usr/bin/env python3

import os
import signal
import sys
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation
from scipy.io import savemat
import numpy as np
from datetime import datetime

class ManualLoggerNode(Node):
    def __init__(self):
        super().__init__("manual_logger_node")

        qos_odom = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        self._cmd_sub = self.create_subscription(
            Twist, "/bebop/cmd_vel", self._cmd_cb, 10)
        
        self._odom_sub = self.create_subscription(
            Odometry, "/bebop/odom_ekf", self._odom_cb, qos_odom)

        self._lock = threading.Lock()
        self._u_current = np.zeros(4)
        self._odom_ready = False
        self._first_odom = True
        self._last_yaw = 0.0
        self._yaw_cont = 0.0
        
        self._pos_i = np.zeros(4)
        self._vel_b = np.zeros(4)
        
        self._data = []
        self._start_time = None
        self._tick_count = 0

        self._ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        base = os.path.expanduser("~/ros2_ws/src/neroControl/data")
        os.makedirs(base, exist_ok=True)
        self._out_mat = os.path.join(base, f"manual_log_{self._ts}.mat")

        self._hz = 10.0
        self._timer = self.create_timer(1.0 / self._hz, self._logger_timer_cb)

        self.get_logger().info("Modo Grabación Manual activo.")
        self.get_logger().info("Esperando odometría y comandos de joystick...")
        self.get_logger().info(f"Guardará en: {self._out_mat}")

        signal.signal(signal.SIGINT, self._signal_handler)

    def _cmd_cb(self, msg):
        with self._lock:
            self._u_current = np.array([
                msg.linear.x,
                msg.linear.y,
                msg.linear.z,
                msg.angular.z
            ])

    def _odom_cb(self, msg):
        q = msg.pose.pose.orientation
        yaw = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz")[2]

        with self._lock:
            if self._first_odom:
                self._first_odom = False
                self._last_yaw = yaw
                self._yaw_cont = yaw
                self._odom_ready = True
                return

            dyaw = np.arctan2(np.sin(yaw - self._last_yaw), np.cos(yaw - self._last_yaw))
            self._yaw_cont += dyaw

            self._pos_i = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
                self._yaw_cont
            ])

            self._vel_b = np.array([
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
                msg.twist.twist.angular.z
            ])

            self._last_yaw = yaw

    def _logger_timer_cb(self):
        if not self._odom_ready:
            return

        if self._start_time is None:
            self._start_time = self.get_clock().now().nanoseconds * 1e-9

        with self._lock:
            u_now = self._u_current.copy()
            pos_now = self._pos_i.copy()
            vel_now = self._vel_b.copy()

        self._data.append({
            "K": self._tick_count,
            "u": u_now,
            "pos": pos_now,
            "vel": vel_now
        })
        self._tick_count += 1

        self.get_logger().info(
            f"Grabando... {len(self._data)} muestras", 
            throttle_duration_sec=3.0
        )

    def _save_mat(self):
        if not self._data:
            self.get_logger().warn("No hay datos para guardar.")
            return
        
        try:
            K = np.array([d["K"] for d in self._data])
            u = np.array([d["u"] for d in self._data])
            p = np.array([d["pos"] for d in self._data])
            v = np.array([d["vel"] for d in self._data])

            savemat(self._out_mat, {
                "K": K,
                "hz": self._hz,
                "u": u,
                "x_i": p[:, 0],
                "y_i": p[:, 1],
                "z_i": p[:, 2],
                "psi": p[:, 3],
                "vx_b": v[:, 0],
                "vy_b": v[:, 1],
                "vz_b": v[:, 2],
                "vpsi": v[:, 3]
            })
            self.get_logger().info(f"Archivo guardado exitosamente: {self._out_mat}")
        except Exception as e:
            self.get_logger().error(f"Error al guardar el archivo .mat: {e}")

    def _signal_handler(self, signum, frame):
        self.get_logger().info("Se detectó interrupción (SIGINT). Deteniendo grabación...")
        self._save_mat()
        sys.exit(0)

def main():
    rclpy.init()
    node = ManualLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._save_mat()
            node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()