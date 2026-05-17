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
from std_msgs.msg import Empty
from scipy.spatial.transform import Rotation
from scipy.io import savemat
import numpy as np
from datetime import datetime
import time as _time

# --- Configuración de Secuencia ---
SEQUENCE    = np.array([1.0, -0.5, 0.8, -1.0, 0.3, 0.7, -0.8, 0.5, -1.0, -0.3, 1.0, -0.7])
N_STEPS     = len(SEQUENCE)
PHASE_DUR   = 4.0
STEP_DUR    = PHASE_DUR / N_STEPS
N_PHASES    = 5
TOTAL_DUR   = PHASE_DUR * N_PHASES

SAMPLE_HZ   = 30.0
CMD_HZ      = 10.0
CMD_EVERY   = int(SAMPLE_HZ / CMD_HZ)
DT_BASE     = 1.0 / SAMPLE_HZ
TOTAL_TICKS = int(TOTAL_DUR * SAMPLE_HZ)

WARMUP_DUR   = 3.0
WARMUP_TICKS = int(WARMUP_DUR * SAMPLE_HZ)

INPUT_NAMES = ["lin_x", "lin_y", "lin_z", "ang_z"]

def seq_value(t):
    idx = min(int((t % PHASE_DUR) / STEP_DUR), N_STEPS - 1)
    return float(SEQUENCE[idx])

def compute_u(tick):
    t_elapsed = tick * DT_BASE
    phase = min(int(t_elapsed / PHASE_DUR), N_PHASES - 1)
    val = seq_value(t_elapsed)
    u = np.zeros(4)
    if phase < 4:
        u[phase] = val
    else:
        u[:] = val
    return np.clip(u, -1.0, 1.0)

class SysIdNode(Node):
    def __init__(self):
        super().__init__("sysid_node")

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self._cmd_pub  = self.create_publisher(Twist, "/bebop/cmd_vel", 10)
        self._land_pub = self.create_publisher(Empty, "/bebop/land", 10)
        self._odom_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self._odom_cb, qos)

        self._lock        = threading.Lock()
        self._odom_ready  = False
        self._first_odom  = True
        self._last_yaw    = 0.0
        self._yaw_cont    = 0.0
        self._last_odom_t = 0.0
        
        # Almacenamiento crudo
        self._pos_i       = np.zeros(4) # x, y, z, psi
        self._vel_b       = np.zeros(4) # vx, vy, vz, vpsi (body)

        self._data               = []
        self._tick               = 0
        self._u_prev             = np.zeros(4)
        self._start_time         = None
        self._done               = False
        self._shutdown_requested = False
        self._in_warmup          = True
        self._warmup_tick        = 0

        self._ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        base = os.path.expanduser("~/ros2_ws/src/neroControl/data")
        os.makedirs(base, exist_ok=True)
        self._out_mat = os.path.join(base, f"sysid_{self._ts}.mat")

        signal.signal(signal.SIGINT,  self._signal_handler)
        self._timer = self.create_timer(DT_BASE, self._timer_cb)

        self.get_logger().info(f"Grabando datos en: {self._out_mat}")

    def _odom_cb(self, msg):
        # Extraer Yaw
        q = msg.pose.pose.orientation
        yaw = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz")[2]
        t_odom = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds * 1e-9

        with self._lock:
            if self._first_odom:
                self._first_odom  = False
                self._last_yaw    = yaw
                self._yaw_cont    = yaw
                self._last_odom_t = t_odom
                self._odom_ready  = True
                return

            # Yaw continuo para evitar saltos de pi a -pi
            dyaw = np.arctan2(np.sin(yaw - self._last_yaw), np.cos(yaw - self._last_yaw))
            self._yaw_cont += dyaw
            
            dt = t_odom - self._last_odom_t
            yaw_rate = (dyaw / dt) if dt > 1e-3 else 0.0

            # Guardar posiciones (Marco Inercial)
            self._pos_i = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
                self._yaw_cont
            ])

            # Guardar velocidades (Marco Body - como vienen del mensaje)
            self._vel_b = np.array([
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
                yaw_rate
            ])

            self._last_yaw = yaw
            self._last_odom_t = t_odom

    def _timer_cb(self):
        if not self._odom_ready or self._done:
            return

        if self._in_warmup:
            self._publish_zero()
            self._warmup_tick += 1
            if self._warmup_tick >= WARMUP_TICKS:
                self._in_warmup = False
                self.get_logger().info("Iniciando secuencia.")
            return

        if self._start_time is None:
            self._start_time = self.get_clock().now().nanoseconds * 1e-9

        tick = self._tick
        if tick >= TOTAL_TICKS:
            self._finish(land=True)
            return

        # Comando a 10Hz
        if tick % CMD_EVERY == 0:
            u = compute_u(tick)
            self._u_prev = u.copy()
            self._publish_cmd(u)

        # Muestreo a 30Hz
        with self._lock:
            pos_i = self._pos_i.copy()
            vel_b = self._vel_b.copy()

        self._data.append({
            "tick": tick,
            "u":    self._u_prev.copy(),
            "pos":  pos_i,
            "vel":  vel_b,
        })
        self._tick += 1

    def _publish_cmd(self, u):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z = map(float, u)
        self._cmd_pub.publish(msg)

    def _publish_zero(self):
        self._cmd_pub.publish(Twist())

    def _save_mat(self):
        if not self._data: return
        try:
            K     = np.array([d["tick"] for d in self._data])
            u     = np.array([d["u"]    for d in self._data])
            pos_i = np.array([d["pos"]  for d in self._data])
            vel_b = np.array([d["vel"]  for d in self._data])

            savemat(self._out_mat, {
                "K":    K,
                "hz":   SAMPLE_HZ,
                "u":    u,
                # Posiciones en marco Inercial
                "x_i":  pos_i[:, 0],
                "y_i":  pos_i[:, 1],
                "z_i":  pos_i[:, 2],
                "psi":  pos_i[:, 3],
                # Velocidades en marco del Cuerpo (Body)
                "vx_b": vel_b[:, 0],
                "vy_b": vel_b[:, 1],
                "vz_b": vel_b[:, 2],
                "vpsi": vel_b[:, 3]
            })
            self.get_logger().info(f"Datos guardados exitosamente en {self._out_mat}")
        except Exception as e:
            self.get_logger().error(f"Error al guardar .mat: {e}")

    def _finish(self, land=True):
        if self._done: return
        self._done = True
        for _ in range(3): self._publish_zero()
        if land: self._land_pub.publish(Empty())
        self._save_mat()
        self._timer.cancel()

    def _signal_handler(self, signum, frame):
        self._finish(land=True)
        rclpy.shutdown()
        sys.exit(0)

def main():
    rclpy.init()
    node = SysIdNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node._done: node._finish(land=True)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == "__main__":
    main()