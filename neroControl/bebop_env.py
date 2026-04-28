import gymnasium as gym
from gymnasium import spaces
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from math import atan2
import time

class BebopEnv(gym.Env):
    def __init__(self):
        super(BebopEnv, self).__init__()
        if not rclpy.ok(): rclpy.init()
        self.node = rclpy.create_node('ia_bebop_env')

        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(4,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(8,), dtype=np.float32)

        self.pub_cmd = self.node.create_publisher(Twist, "/safe_bebop/cmd_vel", 10)
        self.pub_ref = self.node.create_publisher(Float64MultiArray, "/bebop/ref_vec", 10)
        self.sub_odom = self.node.create_subscription(Odometry, "/odometry/filtered", self._odom_cb, 10)
        
        self.current_state = np.zeros(8)
        self.target_state = np.zeros(8)
        self.last_target_change = time.time()
        self.steps_in_episode = 0
        self.dt = 1.0 / 30.0 
        
        self._set_new_target()

    def _odom_cb(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        v, va = msg.twist.twist.linear, msg.twist.twist.angular
        yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.current_state = np.array([p.x, p.y, p.z, yaw, v.x, v.y, v.z, va.z])

    def _set_new_target(self):
        self.target_state[0] = np.random.uniform(-1.5, 1.5)
        self.target_state[1] = np.random.uniform(-1.5, 1.5)
        self.target_state[2] = np.random.uniform(0.8, 2.0)
        self.target_state[3] = np.random.uniform(-1.5, 1.5)
        self.target_state[4:8] = 0.0 
        self.last_target_change = time.time()
        print(f"--- NUEVO OBJETIVO: {self.target_state[:3]} ---")

    def _get_obs(self):
        error = self.target_state - self.current_state
        error[3] = atan2(np.sin(error[3]), np.cos(error[3]))
        return error.astype(np.float32)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.steps_in_episode = 0
        # No reseteamos el target aquí para que mantenga la continuidad
        return self._get_obs(), {}

    def step(self, action):
        start_time = time.time()
        self.steps_in_episode += 1
        action = np.clip(action, -1.0, 1.0)
        
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.linear.z = map(float, action[:3])
        cmd.angular.z = float(action[3])
        self.pub_cmd.publish(cmd)

        ref_msg = Float64MultiArray()
        ref_msg.data = self.target_state.tolist()
        self.pub_ref.publish(ref_msg)

        rclpy.spin_once(self.node, timeout_sec=0.001)

        obs = self._get_obs()
        dist = np.linalg.norm(obs[:3])
        
        # --- LÓGICA DE CAMBIO DE PUNTO SOLICITADA ---
        reached_target = dist < 0.3
        timeout = (time.time() - self.last_target_change) > 15.0

        if reached_target or timeout:
            self._set_new_target()

        # Recompensa con bonus por éxito
        reward = -dist - 0.1 * np.linalg.norm(action)
        if reached_target:
            reward += 10.0 # Gran premio por alcanzar el punto

        # Mantener 30Hz
        elapsed = time.time() - start_time
        if elapsed < self.dt:
            time.sleep(self.dt - elapsed)

        # Terminación y Truncado
        terminated = dist > 30.0 
        truncated = self.steps_in_episode > 1000 # ~33 segundos por episodio
        
        return obs, reward, terminated, truncated, {}