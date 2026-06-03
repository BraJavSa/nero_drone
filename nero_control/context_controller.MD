# Drone Controller Implementation Context

> Pass this file as context whenever implementing a new controller for the Bebop drone platform. It captures all conventions, topics, frequencies, dynamic model, identified parameters, and implementation rules. Gains and control law architecture are **not included** — those depend on the specific controller being designed.

---

## 1. Platform & ROS2 Node Conventions

- **Robot:** Parrot Bebop drone.
- **Middleware:** ROS 2 (Python, `rclpy`).
- **Node base class:** `rclpy.node.Node`.
- **Control loop rate:** `15.0 Hz` (period `Ts = 1/15` s). Always use a `create_timer` callback — do **not** run blocking loops.
- **Spin:** `rclpy.spin(node)` inside `try/except KeyboardInterrupt` with `node.destroy_node()` + `rclpy.shutdown()` in `finally`.

---

## 2. ROS2 Topics

| Direction | Topic | Message Type | Description |
|-----------|-------|--------------|-------------|
| Subscribe | `/bebop/fullodom` | `nav_msgs/Odometry` | Full odometry (position + velocity) |
| Subscribe | `/bebop/ref_vec` | `std_msgs/Float64MultiArray` | Reference vector (see §6) |
| Subscribe | `/bebop/is_flying` | `std_msgs/Bool` | Arming flag — control only when `True` |
| Publish | `/safe_bebop/cmd_vel` | `geometry_msgs/Twist` | Control command to the drone |

> **Safety rule:** If `is_flying` is `False`, publish a zero `Twist` and reset all integrator / memory state. Never publish non-zero commands when the drone is disarmed.

---

## 3. State Vector Convention

All vectors are 4-DOF, always ordered `[x, y, z, ψ]`.

| Symbol | Meaning | ROS2 source |
|--------|---------|-------------|
| `η` (`eta`) | World-frame pose `[x, y, z, ψ]` | `Odometry.pose` |
| `η̇` (`X_dot`) | World-frame velocity `[ẋ, ẏ, ż, ψ̇]` | derived: `J · ν` |
| `ν` (`nu`) | Body-frame velocity `[vx, vy, vz, r]` — surge, sway, heave, yaw-rate | `Odometry.twist` |

- Position `x, y, z` → `pose.pose.position`.
- Yaw `ψ` → extracted from quaternion via `euler_from_quaternion([qx, qy, qz, qw])`.
- Linear velocity `vx, vy, vz` → `twist.twist.linear`.
- Yaw rate `r` → `twist.twist.angular.z`.

**Angle wrapping:** always wrap angular errors with:
```python
wrap_angle(angle) = (angle + π) % (2π) − π
```

---

## 4. Dynamic Model

### 4.1 World-frame (inertial) dynamics

The drone is modelled as a 4-DOF rigid body. In the inertial frame `I`, the translational + yaw acceleration is:

```
η̈ = F1 · U − F2 · η̇
```

where:
- `U ∈ ℝ⁴` is the control input vector.
- `F1 = M̄⁻¹` (inverse of the effective mass-like matrix).
- `F2 = M̄⁻¹ · C̄` (Coriolis/damping term).

### 4.2 Kinematic model

Body-frame velocities relate to world-frame velocities through the yaw-rotation Jacobian:

```
η̇ = J(ψ) · ν
```

```
J(ψ) = [[ cos ψ,  -sin ψ,  0,  0 ],
         [ sin ψ,   cos ψ,  0,  0 ],
         [   0,       0,    1,  0 ],
         [   0,       0,    0,  1 ]]
```

### 4.3 Body-frame dynamics

Substituting the kinematic model into the world-frame dynamics gives the body-frame form:

```
ν̇ = f1 · U − f2 · ν
```

where `f1 = J⁻¹ · F1` and `f2 = J⁻¹ · F2`.

Because `J` is a pure yaw rotation, **`f1` and `f2` are diagonal**:

```
f1 = diag(k1, k3, k5, k7)
f2 = diag(k2, k4, k6, k8)
```

These diagonal parameters `k1 … k8` are identified experimentally.

---

## 5. Identified Parameters

The identified discrete-time matrices (at `Ts = 1/15 s`) are:

```python
F1 = np.array([
    [ 0.988324,  0.014589,  0.000000, -0.009546],
    [-0.019977,  0.986558,  0.000000,  0.005055],
    [ 0.000000,  0.000000,  0.802580,  0.000000],
    [ 0.000000,  0.000000,  0.000000,  0.853101],
])

F2 = np.array([
    [ 0.018878, -0.002861,  0.000000,  0.003410],
    [-0.002938,  0.025773,  0.000000, -0.008888],
    [ 0.000000,  0.000000,  0.122009,  0.000000],
    [ 0.000000,  0.000000,  0.000000,  0.122507],
])
```

These are **world-frame matrices**. When the control law needs body-frame or Jacobian-transformed versions, compute them each control step (since `J` depends on the current `ψ`):

```python
F1_world = J @ F1          # effective world-frame input matrix
F2_eff   = J @ F2 - J_dot  # includes kinematic coupling
F1_inv   = np.linalg.inv(F1_world)
```

---

## 6. Reference Vector Format

The topic `/bebop/ref_vec` carries a flat `Float64MultiArray` with **exactly 12 elements**:

| Indices | Symbol | Meaning |
|---------|--------|---------|
| `[0:4]` | `η_d` | Desired pose `[xd, yd, zd, ψd]` |
| `[4:8]` | `ν_d` | Desired velocity (world frame) |
| `[8:12]` | `α_d` | Desired acceleration (feedforward) |

If `len(msg.data) < 12`, discard the message silently.

---

## 7. Control Output Format

Publish on `/safe_bebop/cmd_vel` as `geometry_msgs/Twist`:

| Field | Maps to |
|-------|---------|
| `linear.x` | `U[0]` — surge |
| `linear.y` | `U[1]` — sway |
| `linear.z` | `U[2]` — heave |
| `angular.z` | `U[3]` — yaw rate |
| `angular.x`, `angular.y` | Always `0.0` |

**Saturation:** clip the 4-DOF output to `[−1, 1]` per channel before publishing:
```python
U_body = np.clip(Ud, -np.ones(4), np.ones(4))
```

---

## 8. Helper Utilities (always include in every controller file)

```python
def euler_from_quaternion(q):
    """q = [x, y, z, w] → (roll, pitch, yaw)"""
    x, y, z, w = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    sinp = np.clip(2.0 * (w * y - z * x), -1.0, 1.0)
    pitch = np.arcsin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

def wrap_angle(angle: float) -> float:
    return (angle + np.pi) % (2.0 * np.pi) - np.pi

def jacobian(psi: float) -> np.ndarray:
    c, s = np.cos(psi), np.sin(psi)
    return np.array([[ c, -s, 0, 0],
                     [ s,  c, 0, 0],
                     [ 0,  0, 1, 0],
                     [ 0,  0, 0, 1]])

def jacobian_inv(psi: float) -> np.ndarray:
    c, s = np.cos(psi), np.sin(psi)
    return np.array([[ c,  s, 0, 0],
                     [-s,  c, 0, 0],
                     [ 0,  0, 1, 0],
                     [ 0,  0, 0, 1]])

def jacobian_dot(psi: float, psi_dot: float) -> np.ndarray:
    c, s = np.cos(psi), np.sin(psi)
    return psi_dot * np.array([[-s, -c, 0, 0],
                                [ c, -s, 0, 0],
                                [ 0,  0, 0, 0],
                                [ 0,  0, 0, 0]])

def saturate(v: np.ndarray, vmax: np.ndarray) -> np.ndarray:
    return np.clip(v, -vmax, vmax)
```

---

## 9. Node Initialisation Checklist

```
[ ] self.Ts = 1.0 / 15.0
[ ] self.eta  = np.zeros(4)   # world-frame pose
[ ] self.nu   = np.zeros(4)   # body-frame velocity
[ ] self.eta_d   = np.zeros(4)
[ ] self.nu_d    = np.zeros(4)
[ ] self.alpha_d = np.zeros(4)
[ ] self.is_flying    = False
[ ] self.odom_received = False
[ ] self.ref_received  = False
[ ] Subscribe: /bebop/fullodom, /bebop/ref_vec, /bebop/is_flying
[ ] Publish:   /safe_bebop/cmd_vel
[ ] self.create_timer(self.Ts, self.control_loop)
```

---

## 10. Control Loop Checklist

```
[ ] if not is_flying  → _publish_zero(), _reset_memory(), return
[ ] if not odom_received or not ref_received → _publish_zero(), return
[ ] Copy eta, nu at start of loop (never mutate stored state mid-step)
[ ] Recompute J, J_dot, J_inv every step (ψ changes continuously)
[ ] Wrap all yaw errors with wrap_angle()
[ ] Saturate output to ±1 before publishing
[ ] Update any memory variables (prev values, integrators) at end of step
```

---

## 11. Memory Reset on Disarm

`_reset_memory()` must zero **all** variables that carry state between steps. At minimum:
- Any `*_prev` variables (e.g. previous setpoint, previous error).
- Any integrator accumulators.

This is called unconditionally every loop iteration while `is_flying = False`.

---

## 12. Imports

```python
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, Bool
```