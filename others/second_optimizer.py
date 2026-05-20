#!/usr/bin/env python3
# Secondary script for executing iterative parameter optimization.


import math
import torch


device = torch.device("cpu")
DT = 1.0 / 30.0
T_TOTAL = 40.0
OMEGA = 2.0 * math.pi / T_TOTAL
U_SAT = 1.0



MODEL_SIMP = torch.tensor([
    0.8417, 0.18227,
    0.8354, 0.17095,
    3.966,  4.001,
    9.8524, 4.7295
], dtype=torch.float32, device=device)

Ku_diag = MODEL_SIMP[[0, 2, 4, 6]]
Kv_diag = MODEL_SIMP[[1, 3, 5, 7]]

Ku = torch.diag(Ku_diag)
Kv = torch.diag(Kv_diag)



def wrap_angle(angle: torch.Tensor) -> torch.Tensor:
    return torch.atan2(torch.sin(angle), torch.cos(angle))


def rotation_F(yaw: torch.Tensor) -> torch.Tensor:
    c = torch.cos(yaw)
    s = torch.sin(yaw)
    F = torch.zeros(4, 4, dtype=torch.float32, device=device)
    F[0, 0] = c
    F[0, 1] = -s
    F[1, 0] = s
    F[1, 1] = c
    F[2, 2] = 1.0
    F[3, 3] = 1.0
    return F



def ref_trajectory(t: torch.Tensor):
    w = OMEGA

    x = 1.5 * torch.sin(w * t)
    y = 1.5 * torch.sin(w * t) * torch.cos(w * t)
    z = 0.5 + 0.3 * torch.sin(0.5 * w * t)

    dx = 1.5 * w * torch.cos(w * t)
    dy = 1.5 * w * torch.cos(2.0 * w * t)
    dz = 0.15 * w * torch.cos(0.5 * w * t)

    yaw = torch.atan2(dy, dx)

    ddx = -1.5 * (w ** 2) * torch.sin(w * t)
    ddy = -3.0 * (w ** 2) * torch.sin(2.0 * w * t)

    denom = dx * dx + dy * dy + 1e-6
    yaw_dot = (dx * ddy - dy * ddx) / denom

    Xd = torch.stack([x, y, z, yaw])
    dXd = torch.stack([dx, dy, dz, yaw_dot])
    return Xd, dXd



def dynamics_step(x, xdot, u, dt):
    yaw = x[3]
    F = rotation_F(yaw)
    xddot = F @ (Ku @ u) - (Kv @ xdot)

    xdot_new = xdot + xddot * dt
    x_new = x + xdot_new * dt
    return x_new, xdot_new



def compute_odom_from_state(x, xdot):
    return x.clone(), xdot.clone()



def inverse_dynamic_controller(X_meas, dX_meas, Xd, dXd, gains, Ucw_prev, dt):

    g = gains
    Ksp = torch.diag(g[0:4])
    Ksd = torch.diag(g[4:8])
    Kp  = torch.diag(g[8:12])

    X = X_meas
    dX = dX_meas

    Xtil_raw = Xd - X
    yaw_err = wrap_angle(Xtil_raw[3])
    Xtil = torch.stack([Xtil_raw[0], Xtil_raw[1], Xtil_raw[2], yaw_err])

    Ucw = dXd + Ksp @ torch.tanh(Kp @ Xtil)
    dUcw = (Ucw - Ucw_prev) / dt

    yaw = X[3]
    F = rotation_F(yaw)
    M = F @ Ku
    rhs = dUcw + Ksd @ (Ucw - dX) + Kv @ dX
    Udw = torch.linalg.solve(M, rhs)

    Ud = torch.zeros(6, dtype=torch.float32, device=device)
    Ud[0:3] = Udw[0:3]
    Ud[5] = Udw[3]

    return Ud, Ucw



def simulate_episode(gains: torch.Tensor) -> torch.Tensor:

    steps = int(T_TOTAL / DT)
    odom_period = 6

    x = torch.zeros(4, device=device)
    xdot = torch.zeros(4, device=device)

    X_meas, dX_meas = compute_odom_from_state(x, xdot)
    Ucw_prev = torch.zeros(4, device=device)

    cost = torch.tensor(0.0, device=device)
    n_odom_samples = 0

    for k in range(steps):
        t = torch.tensor(k * DT, device=device)
        Xd, dXd = ref_trajectory(t)

        if k % odom_period == 0:
            X_meas, dX_meas = compute_odom_from_state(x, xdot)

            pos_err = Xd[0:3] - X_meas[0:3]
            yaw_err = wrap_angle(Xd[3] - X_meas[3])

            loss_track = (pos_err**2).sum() + 0.5*(yaw_err**2)
            loss_vel = 0.1 * (dX_meas[0:3]**2).sum()

            loss = loss_track + loss_vel
            cost = cost + loss
            n_odom_samples += 1

        Ud, Ucw_prev = inverse_dynamic_controller(
            X_meas, dX_meas, Xd, dXd, gains, Ucw_prev, DT
        )

        Ud_clipped = torch.clamp(Ud, -U_SAT, U_SAT)

        cost = cost + 0.01*(Ud_clipped[0:3]**2).sum()

        yaw = x[3]
        F = rotation_F(yaw)
        xddot_proxy = F @ (Ku @ Ud_clipped[0:4]) - Kv @ xdot
        cost = cost + 0.005*(xddot_proxy[0:3]**2).sum()

        u_dyn = torch.stack([
            Ud_clipped[0],
            Ud_clipped[1],
            Ud_clipped[2],
            Ud_clipped[5]
        ])

        x, xdot = dynamics_step(x, xdot, u_dyn, DT)

    if n_odom_samples > 0:
        cost = cost / n_odom_samples

    return cost



def main():

    init_gains = torch.tensor([0.2948657274246216, 0.282520592212677, 0.33964332938194275, 2.4078164100646973, 4.946599960327148, 4.323536396026611, 6.013118267059326, 10.796932220458984, 0.29646456241607666, 0.28467491269111633, 3.555035352706909, 1.9480745792388916] dtype=torch.float32, device=device)

    gains = torch.nn.Parameter(init_gains.clone())
    optimizer = torch.optim.Adam([gains], lr=1e-2)

    n_epochs = 200

    for epoch in range(n_epochs):
        optimizer.zero_grad()
        cost = simulate_episode(gains)
        cost.backward()
        optimizer.step()

        if (epoch + 1) % 10 == 0 or epoch == 0:
            print("Epoch =", epoch+1)
            print("Cost =", float(cost))
            print("Gains =", gains.detach().cpu().tolist())

    print("# --- Optimización terminada ---")
    print("Gains_opt =", gains.detach().cpu().tolist())


if __name__ == "__main__":
    main()
