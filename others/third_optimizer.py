#!/usr/bin/env python3
# Tertiary optimization logic script.


import math
import time
import numpy as np
import torch
from torch.optim import Adam


DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
DT_MASTER = 1/150
DT_DYN = 1/50
DT_REF = 1/30
DT_CTRL = 1/10

EPISODE_TIME = 12.0 * 5
N_STEPS = int(EPISODE_TIME / DT_MASTER)

W_ERR = 1.0
W_DU = 0.2
W_OVERSHOOT = 2.0
W_TSETTLE = 0.3
W_SAT = 0.5
W_OSC = 0.2
W_YAW_SMOOTH = 0.3

Z_TOL = 0.02

MODEL_SIMP = np.array([
    0.8417, 0.18227,
    0.8354, 0.17095,
    3.966,  4.001,
    9.8524, 4.7295
], dtype=np.float32)

Ku_np = np.diag([MODEL_SIMP[0], MODEL_SIMP[2], MODEL_SIMP[4], MODEL_SIMP[6]])
Kv_np = np.diag([MODEL_SIMP[1], MODEL_SIMP[3], MODEL_SIMP[5], MODEL_SIMP[7]])

Ku = torch.tensor(Ku_np, dtype=torch.float32, device=DEVICE)
Kv = torch.tensor(Kv_np, dtype=torch.float32, device=DEVICE)

BASE_GAINS = np.array([
    1.2, 1.2, 3.0, 1.5,
    1.0, 1.0, 1.8, 1.2,
    1.7, 1.7, 1.0, 1.5
], dtype=np.float32)

DELTA_SCALE = np.array([
    0.3, 0.3, 0.5, 0.4,
    0.3, 0.3, 0.5, 0.4,
    0.4, 0.4, 0.3, 0.4
], dtype=np.float32)

BASE_GAINS_T = torch.tensor(BASE_GAINS, dtype=torch.float32, device=DEVICE)
DELTA_SCALE_T = torch.tensor(DELTA_SCALE, dtype=torch.float32, device=DEVICE)


def build_reference_points():
    L = 1.5
    points = np.array([
        [0.0, 0.0, 1.5],
        [L/2, L/2, 1.7],
        [-L/2, L/2, 1.4],
        [-L/2, -L/2, 1.8],
        [L/2, -L/2, 1.2]
    ], dtype=np.float32)
    yaws = np.deg2rad([0, 50, 150, 180, 210]).astype(np.float32)
    return points, yaws

POINTS, YAWS = build_reference_points()
HOLD_TIME = 12.0

def get_ref_at_time(t):
    idx = int(t // HOLD_TIME)
    if idx >= len(POINTS):
        idx = len(POINTS) - 1
    pos = POINTS[idx]
    yaw = YAWS[idx]
    Xd = torch.tensor([pos[0], pos[1], pos[2], yaw], dtype=torch.float32, device=DEVICE)
    dXd = torch.zeros(4, dtype=torch.float32, device=DEVICE)
    return Xd, dXd


def dynamics_step(x, xdot, u):
    yaw = x[3]
    c = torch.cos(yaw)
    s = torch.sin(yaw)

    zero = torch.tensor(0.0, device=DEVICE)
    one = torch.tensor(1.0, device=DEVICE)

    F = torch.stack([
        torch.stack([c,   -s,  zero, zero]),
        torch.stack([s,    c,  zero, zero]),
        torch.stack([zero, zero, one,  zero]),
        torch.stack([zero, zero, zero, one])
    ])

    xddot = F @ (Ku @ u) - Kv @ xdot
    xdot_new = xdot + xddot * DT_DYN
    x_new = x + xdot_new * DT_DYN

    z = torch.clamp(x_new[2], min=0.0)
    x_new2 = torch.stack([x_new[0], x_new[1], z, x_new[3]])

    return x_new2, xdot_new


def controller_step(x, xdot, Xd, dXd, gains, prev_Ur):
    Ksp = torch.diag(gains[0:4])
    Ksd = torch.diag(gains[4:8])
    Kp  = torch.diag(gains[8:12])

    X = x
    dX = xdot

    base_err = Xd - X
    yaw_err = base_err[3]
    wrap = (torch.abs(yaw_err) > math.pi).float()
    yaw_err_wrapped = yaw_err - 2*math.pi*torch.sign(yaw_err)*wrap

    Xtil = torch.stack([base_err[0], base_err[1], base_err[2], yaw_err_wrapped])

    Ucw = dXd + Ksp @ torch.tanh(Kp @ Xtil)
    dUcw = (Ucw - prev_Ur) / DT_CTRL
    Ur = Ucw

    psi = X[3]
    c = torch.cos(psi)
    s = torch.sin(psi)
    zero = torch.tensor(0.0, device=DEVICE)
    one = torch.tensor(1.0, device=DEVICE)

    F = torch.stack([
        torch.stack([c,   -s,  zero, zero]),
        torch.stack([s,    c,  zero, zero]),
        torch.stack([zero, zero, one,  zero]),
        torch.stack([zero, zero, zero, one])
    ])

    M = F @ Ku
    Minv = torch.inverse(M)

    Udw = Minv @ (dUcw + Ksd @ (Ucw - dX) + Kv @ dX)

    u_body = torch.stack([
        torch.clamp(Udw[0], -1.0, 1.0),
        torch.clamp(Udw[1], -1.0, 1.0),
        torch.clamp(Udw[2], -1.0, 1.0),
        torch.clamp(Udw[3], -1.0, 1.0)
    ])

    return u_body, Ur


def compute_cost(traj_x, traj_xd, traj_u):
    e = traj_x - traj_xd
    J_err = (e**2).sum(dim=1).mean()

    du = traj_u[1:] - traj_u[:-1]
    J_du = (du**2).sum(dim=1).mean()

    z = traj_x[:,2]
    z_ref = traj_xd[:,2]
    overshoot = torch.clamp(z - z_ref, min=0.0)
    J_ov = overshoot.max()

    ez = torch.abs(z - z_ref)
    settled = ez < Z_TOL
    t_settle = EPISODE_TIME
    for i in range(len(ez)):
        if settled[i:].all():
            t_settle = i * DT_MASTER
            break
    J_tset = torch.tensor(t_settle / EPISODE_TIME, device=DEVICE)

    sat_mask = (torch.abs(traj_u) > 0.9).float()
    J_sat = sat_mask.mean()

    dx = traj_x[1:] - traj_x[:-1]
    J_osc = (dx**2).sum(dim=1).mean()

    yaw = traj_x[:,3]
    dyaw = yaw[1:] - yaw[:-1]
    J_yaw = (dyaw**2).mean()

    J = (
        W_ERR * J_err +
        W_DU * J_du +
        W_OVERSHOOT * J_ov +
        W_TSETTLE * J_tset +
        W_SAT * J_sat +
        W_OSC * J_osc +
        W_YAW_SMOOTH * J_yaw
    )

    return J


def rollout(theta):
    gains = BASE_GAINS_T + DELTA_SCALE_T * torch.tanh(theta)

    x = torch.tensor([0.0,0.0,1.2,0.0], dtype=torch.float32, device=DEVICE)
    xdot = torch.zeros(4, dtype=torch.float32, device=DEVICE)
    Ur = torch.zeros(4, dtype=torch.float32, device=DEVICE)
    u_body = torch.zeros(4, dtype=torch.float32, device=DEVICE)

    traj_x = []
    traj_xd = []
    traj_u = []

    Xd, dXd = get_ref_at_time(0.0)

    dyn_t = 0.0
    ref_t = 0.0
    ctrl_t = 0.0

    for step in range(N_STEPS):
        t = step * DT_MASTER
        dyn_t += DT_MASTER
        ref_t += DT_MASTER
        ctrl_t += DT_MASTER

        if ref_t >= DT_REF:
            Xd, dXd = get_ref_at_time(t)
            ref_t = 0.0

        if ctrl_t >= DT_CTRL:
            u_body, Ur = controller_step(x, xdot, Xd, dXd, gains, Ur)
            ctrl_t = 0.0

        if dyn_t >= DT_DYN:
            x, xdot = dynamics_step(x, xdot, u_body)
            dyn_t = 0.0

        traj_x.append(x)
        traj_xd.append(Xd)
        traj_u.append(u_body)

    traj_x = torch.stack(traj_x)
    traj_xd = torch.stack(traj_xd)
    traj_u = torch.stack(traj_u)

    J = compute_cost(traj_x, traj_xd, traj_u)
    return J, gains


def train(num_iters=50, lr=1e-1):
    theta = torch.zeros(12, dtype=torch.float32, device=DEVICE, requires_grad=True)
    opt = Adam([theta], lr=lr)

    best = BASE_GAINS.copy().tolist()
    bestJ = float("inf")

    for it in range(num_iters):
        opt.zero_grad()
        J, gains = rollout(theta)
        J.backward()
        opt.step()

        gains_np = gains.detach().cpu().numpy().tolist()

        if J.item() < bestJ:
            bestJ = J.item()
            best = gains_np

        print(f"[{it+1:03d}] J={J.item():.4f}  BEST_J={bestJ:.4f}")
        print("gains =", best)

    print("\nMEJORES GANANCIAS FINALES:")
    print("gains =", best)
    return best


if __name__ == "__main__":
    t0 = time.time()
    best = train(50, 1e-1)
    print("Tiempo:", time.time()-t0)
