import time
import numpy as np
from dataclasses import dataclass
from .parameters import DroneParams

@dataclass
class PoseState:
    # X = [x y z psi], dX = [dx dy dz dpsi]
    X:   np.ndarray  # shape (4,)
    dX:  np.ndarray  # shape (4,)
    Xd:  np.ndarray  # shape (4,)
    dXd: np.ndarray  # shape (4,)
    d2Xd: np.ndarray # shape (4,)

def _wrap_angle_pi(a: float) -> float:
    # Wrap to [-pi, pi]
    a = (a + np.pi) % (2*np.pi) - np.pi
    return a

class InverseDynamicCompensator:
    def __init__(self, params: DroneParams, gains=None):
        self.p = params

        if gains is None:
            gains = np.array([
                2,   2,   3,   1.5,   # Ksp diag
                2,   2,   1.8, 5,     # Ksd diag
                1,   1,   1,   1.5,   # Kp  diag
                1,   1,   1,   1      # Kd  diag
            ], dtype=float)
        assert gains.size == 16
        # Gains matrices
        self.Ksp = np.diag(gains[0:4])
        self.Ksd = np.diag(gains[4:8])
        self.Kp  = np.diag(gains[8:12])
        self.Kd  = np.diag(gains[12:16])

        # Ku, Kv from Model_simp to match MATLAB lines
        ms = self.p.Model_simp
        Ku_diag = np.array([ms[0], ms[2], ms[4], ms[6]])
        Kv_diag = np.array([ms[1], ms[3], ms[5], ms[7]])
        self.Ku = np.diag(Ku_diag)
        self.Kv = np.diag(Kv_diag)

        # Memory for Ucw and timing (mimic tic/toc)
        self._Ucw_prev = np.zeros((4,))
        self._t_prev   = time.time()

    def step(self, X_full: np.ndarray, dX_full: np.ndarray,
             Xd_full: np.ndarray, dXd_full: np.ndarray, d2Xd_full: np.ndarray):
        """
        Inputs (full 12-vectors like MATLAB), we internally pick [x y z psi] and [dx dy dz dpsi]
        X_full:   shape (12,)
        dX_full:  shape (12,)
        Xd_full:  shape (12,)
        dXd_full: shape (12,)
        d2Xd_full:shape (12,)
        Returns Ud (6,) mapped to cmd_vel: [vx vy vz wx wy wz] with wx=wy=0, wz=Udw(4)
        """
        # Map to reduced states
        X   = np.array([X_full[0], X_full[1], X_full[2], X_full[5]], dtype=float)
        dX  = np.array([dX_full[6], dX_full[7], dX_full[8], dX_full[11]], dtype=float)
        Xd  = np.array([Xd_full[0], Xd_full[1], Xd_full[2], Xd_full[5]], dtype=float)
        dXd = np.array([dXd_full[6], dXd_full[7], dXd_full[8], dXd_full[11]], dtype=float)
        d2Xd= np.array([d2Xd_full[6], d2Xd_full[7], d2Xd_full[8], d2Xd_full[11]], dtype=float)

        # Error
        Xtil  = Xd - X
        Xtil[3] = _wrap_angle_pi(Xtil[3])  # wrap yaw error
        dXtil = dXd - dX

        # Kinematic control
        Ucw = dXd + self.Ksp @ np.tanh(self.Kp @ Xtil)

        # dUcw with dt
        t_now = time.time()
        dt = max(1e-6, t_now - self._t_prev)
        dUcw = (Ucw - self._Ucw_prev) / dt
        self._Ucw_prev = Ucw
        self._t_prev   = t_now

        # Direct kinematics transform F
        psi = X[3]
        F = np.array([
            [ np.cos(psi), -np.sin(psi), 0, 0],
            [ np.sin(psi),  np.cos(psi), 0, 0],
            [ 0,            0,           1, 0],
            [ 0,            0,           0, 1]
        ], dtype=float)

        # Dynamic compensator: (F*Ku)\(dUcw + Ksd*(Ucw - dX) + Kv*dX)
        A = F @ self.Ku
        rhs = dUcw + self.Ksd @ (Ucw - dX) + self.Kv @ dX
        Udw = np.linalg.solve(A, rhs)  # 4x1

        # Map to cmd_vel Ud (6): [vx vy vz 0 0 wz]
        Ud = np.zeros((6,))
        Ud[0] = Udw[0]
        Ud[1] = Udw[1]
        Ud[2] = Udw[2]
        Ud[5] = Udw[3]
        return Ud
