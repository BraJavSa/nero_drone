#!/usr/bin/env python3
# Velocity test trajectory for Bebop drone using trapezoidal profiles with cosine ramps
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
import numpy as np
import time

V_XY    = 1.0
V_Z     = 0.5
V_YAW   = np.deg2rad(30.0)
T_RAMP   = 5.0
T_CRUISE = 3.0
T_HOVER  = 2.0
T_WAIT   = 5.0
PUBLISH_HZ = 30.0

def cosine_ramp(t: float, t0: float, t1: float) -> float:
    if t <= t0: return 0.0
    if t >= t1: return 1.0
    x = (t - t0) / (t1 - t0)
    return 0.5 * (1.0 - np.cos(np.pi * x))

def velocity_profile(t: float, v_peak: float, t_ramp: float, t_cruise: float) -> float:
    t_brake_start = t_ramp + t_cruise
    t_end         = t_brake_start + t_ramp
    if t < 0.0: return 0.0
    elif t <= t_ramp: return v_peak * cosine_ramp(t, 0.0, t_ramp)
    elif t <= t_brake_start: return v_peak
    elif t <= t_end: return v_peak * (1.0 - cosine_ramp(t, t_brake_start, t_end))
    else: return 0.0

def segment_duration(t_ramp: float, t_cruise: float) -> float:
    return 2.0 * t_ramp + t_cruise

SEGMENTS = [
    ('+X forward',   V_XY,   0),
    ('-X backward', -V_XY,   0),
    ('+Y right',     V_XY,   1),
    ('-Y left',     -V_XY,   1),
    ('+Z up',        V_Z,    2),
    ('-Z down',     -V_Z,    2),
    ('+Yaw CW',      V_YAW,  3),
    ('-Yaw CCW',    -V_YAW,  3),
]

class VelocityTrajectory(Node):
    def __init__(self):
        super().__init__('bebop_velocity_trajectory')
        self.is_flying = False
        self.sub_flying = self.create_subscription(Bool, '/bebop/is_flying', self._flying_cb, 10)
        self.pub_ref = self.create_publisher(Float64MultiArray, '/bebop/ref_vec', 10)
        seg_dur = segment_duration(T_RAMP, T_CRUISE)
        total   = len(SEGMENTS) * (seg_dur + T_HOVER)
        self.get_logger().info(f"Trajectory ready | Ramp={T_RAMP:.1f}s Cruise={T_CRUISE:.1f}s Hover={T_HOVER:.1f}s")
        self.get_logger().info(f"Total duration: ~{total:.0f}s ({len(SEGMENTS)} segments)")
        self.get_logger().info(f"Peak velocities: XY={V_XY:.2f}m/s Z={V_Z:.2f}m/s Yaw={np.rad2deg(V_YAW):.1f}deg/s")

    def _flying_cb(self, msg: Bool):
        self.is_flying = msg.data

    def _publish(self, vx=0.0, vy=0.0, vz=0.0, dyaw=0.0):
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0, 0.0, 0.0, float(vx), float(vy), float(vz), float(dyaw)]
        self.pub_ref.publish(msg)

    def _hover(self, duration: float):
        dt = 1.0 / PUBLISH_HZ
        steps = int(duration * PUBLISH_HZ)
        for _ in range(steps):
            if not rclpy.ok(): return
            rclpy.spin_once(self, timeout_sec=0.0)
            self._publish()
            time.sleep(dt)

    def _run_segment(self, name: str, v_peak: float, axis: int):
        dt = 1.0 / PUBLISH_HZ
        dur = segment_duration(T_RAMP, T_CRUISE)
        steps = int(dur * PUBLISH_HZ)
        vel_cmd = [0.0, 0.0, 0.0, 0.0]
        log_every = max(1, int(0.5 * PUBLISH_HZ))
        self.get_logger().info(f"  -> {name:18s}  v_peak={v_peak:+.2f}  duration={dur:.1f}s")
        for i in range(steps + 1):
            if not rclpy.ok(): return
            t = i * dt
            v = velocity_profile(t, v_peak, T_RAMP, T_CRUISE)
            vel_cmd[axis] = v
            rclpy.spin_once(self, timeout_sec=0.0)
            self._publish(*vel_cmd)
            time.sleep(dt)
            if i % log_every == 0:
                phase = ('ACC' if t < T_RAMP else 'CRUISE' if t < T_RAMP + T_CRUISE else 'BRAKE')
                self.get_logger().info(f"    t={t:4.1f}s  v={v:+.3f}  [{phase}]")
        vel_cmd[axis] = 0.0
        self._publish(*vel_cmd)

    def run(self):
        self.get_logger().info(f"Waiting for /bebop/is_flying (max {T_WAIT:.0f}s)...")
        t0 = time.time()
        while not self.is_flying:
            rclpy.spin_once(self, timeout_sec=0.05)
            self._publish()
            if time.time() - t0 > T_WAIT:
                self.get_logger().error("Timeout: Drone is not flying. Check /bebop/is_flying.")
                return
        self.get_logger().info("Drone in flight. Starting trajectory...")
        self.get_logger().info("Initial hover 2.0s")
        self._hover(2.0)
        for idx, (name, v_peak, axis) in enumerate(SEGMENTS):
            if not rclpy.ok(): break
            self.get_logger().info(f"[{idx+1}/{len(SEGMENTS)}] {name}")
            self._run_segment(name, v_peak, axis)
            self.get_logger().info(f"  hover {T_HOVER:.1f}s")
            self._hover(T_HOVER)
        self.get_logger().info("Trajectory completed. Final hover 3.0s.")
        self._hover(3.0)
        self.get_logger().info("Done.")

def main(args=None):
    rclpy.init(args=args)
    node = VelocityTrajectory()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted by user. Sending safety hover...")
        for _ in range(int(PUBLISH_HZ * 2)):
            node._publish()
            time.sleep(1.0 / PUBLISH_HZ)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()