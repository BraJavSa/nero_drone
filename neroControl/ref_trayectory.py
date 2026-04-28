#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import numpy as np
import time

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__("trajectory_ref_publisher")

        # Publishers
        self.pub_ref = self.create_publisher(Float64MultiArray, "/bebop/ref_vec", 10)
        self.tf_br = TransformBroadcaster(self)

        # TF buffer/listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- PARAMETROS DE TRAYECTORIA CONFIGURABLES ---
        self.dt = 1.0 / 30.0     
        
        # Reducimos w para que dure más (ej: 0.1 rad/s significa una vuelta completa en ~62 segundos)
        self.w = 0.1             
        
        # Aumentamos la amplitud (antes era 0.8, ahora 1.5 para que sea más ancha y larga)
        self.amplitude_x = 1.5   
        self.amplitude_y = 1.5   
        
        # Altura base y oscilación en Z
        self.z_base = 1.5
        self.z_amp = 0.5
        
        # -----------------------------------------------

        self.start_time = time.time()
        
        # State for Yaw Unwrapping
        self.first_sample = True
        self.last_raw_yaw = 0.0
        self.last_unwrapped_yaw = 0.0

        # Timer
        self.timer = self.create_timer(self.dt, self.timer_cb)

        self.get_logger().info(f"Trayectoria expandida iniciada: Amplitud={self.amplitude_x}, w={self.w}")

    def timer_cb(self):
        t = time.time() - self.start_time

        # 1. Calcular posición actual en INITIAL_FRAME (Dimensiones aumentadas)
        x = self.amplitude_x * np.sin(self.w * t)
        y = self.amplitude_y * np.sin(self.w * t) * np.cos(self.w * t)
        z = self.z_base + self.z_amp * np.sin(0.5 * self.w * t)
        pos_i = np.array([x, y, z])

        # 2. Diferenciación para velocidad tangencial en INITIAL_FRAME
        t_prev = t - self.dt
        x_prev = self.amplitude_x * np.sin(self.w * t_prev)
        y_prev = self.amplitude_y * np.sin(self.w * t_prev) * np.cos(self.w * t_prev)
        z_prev = self.z_base + self.z_amp * np.sin(0.5 * self.w * t_prev)

        dx = (x - x_prev) / self.dt
        dy = (y - y_prev) / self.dt
        dz = (z - z_prev) / self.dt
        vel_i = np.array([dx, dy, dz])

        # 3. Lógica de Yaw Continuo
        current_raw_yaw = np.arctan2(dy, dx)
        
        if self.first_sample:
            self.last_raw_yaw = current_raw_yaw
            self.last_unwrapped_yaw = current_raw_yaw
            self.first_sample = False

        delta_yaw = current_raw_yaw - self.last_raw_yaw
        delta_yaw = (delta_yaw + np.pi) % (2 * np.pi) - np.pi
        
        current_unwrapped_yaw = self.last_unwrapped_yaw + delta_yaw
        wyaw = (current_unwrapped_yaw - self.last_unwrapped_yaw) / self.dt

        self.last_raw_yaw = current_raw_yaw
        self.last_unwrapped_yaw = current_unwrapped_yaw

        # 4. Transformar de INITIAL_FRAME a ODOM
        try:
            tf_oi = self.tf_buffer.lookup_transform(
                "odom", "initial_frame", rclpy.time.Time()
            )
        except Exception:
            return

        tx = tf_oi.transform.translation.x
        ty = tf_oi.transform.translation.y
        tz = tf_oi.transform.translation.z
        qx = tf_oi.transform.rotation.x
        qy = tf_oi.transform.rotation.y
        qz = tf_oi.transform.rotation.z
        qw = tf_oi.transform.rotation.w

        R = self.quaternion_to_matrix(qx, qy, qz, qw)
        base_yaw = self.quaternion_yaw(qx, qy, qz, qw)

        pos_o = R.dot(pos_i) + np.array([tx, ty, tz])
        vel_o = R.dot(vel_i)
        yaw_o = current_unwrapped_yaw + base_yaw

        # 5. Publicar Vector de Referencia
        msg = Float64MultiArray()
        msg.data = [
            float(pos_o[0]), float(pos_o[1]), float(pos_o[2]), float(yaw_o),
            float(vel_o[0]), float(vel_o[1]), float(vel_o[2]), float(wyaw)
        ]
        self.pub_ref.publish(msg)

        # 6. Publicar TF odom -> ref
        qz_ref = np.sin(yaw_o * 0.5)
        qw_ref = np.cos(yaw_o * 0.5)

        tmsg = TransformStamped()
        tmsg.header.stamp = self.get_clock().now().to_msg()
        tmsg.header.frame_id = "odom"
        tmsg.child_frame_id = "ref"
        tmsg.transform.translation.x = float(pos_o[0])
        tmsg.transform.translation.y = float(pos_o[1])
        tmsg.transform.translation.z = float(pos_o[2])
        tmsg.transform.rotation.z = qz_ref
        tmsg.transform.rotation.w = qw_ref

        self.tf_br.sendTransform(tmsg)

    def quaternion_to_matrix(self, x, y, z, w):
        R = np.zeros((3, 3))
        R[0, 0] = 1 - 2*(y*y + z*z)
        R[0, 1] = 2*(x*y - z*w)
        R[0, 2] = 2*(x*z + y*w)
        R[1, 0] = 2*(x*y + z*w)
        R[1, 1] = 1 - 2*(x*x + z*z)
        R[1, 2] = 2*(y*z - x*w)
        R[2, 0] = 2*(x*z - y*w)
        R[2, 1] = 2*(y*z + x*w)
        R[2, 2] = 1 - 2*(x*x + y*y)
        return R

    def quaternion_yaw(self, x, y, z, w):
        return np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

def main():
    rclpy.init()
    node = TrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()