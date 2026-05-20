#!/usr/bin/env python3

# Interactive GUI with sliders for tuning PID gains on the simulated dynamic model.

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from mpl_toolkits.mplot3d import Axes3D


RATE_HZ = 15.0
Ts = 1.0 / RATE_HZ


F1 = np.diag([
    0.988324,
    0.986558,
    0.802580,
    0.853101
])

F2 = np.diag([
    0.018878,
    0.025773,
    0.122009,
    0.122507
])

U_MAX = np.array([1.0, 1.0, 1.0, 1.0])


WAYPOINTS = np.array([

    [ 1.0,  1.0,  1.0, 0.0],

    [-1.0,  1.0,  1.0, np.pi/2],

    [-1.0, -1.0, -1.0, np.pi],

    [ 1.0, -1.0,  1.0, -np.pi/2],

])

TIME_PER_POINT = 15.0

TOTAL_TIME = TIME_PER_POINT * len(WAYPOINTS)

N = int(TOTAL_TIME * RATE_HZ)


def wrap_angle(a):
    return (a + np.pi) % (2*np.pi) - np.pi


def jacobian(psi):

    c = np.cos(psi)
    s = np.sin(psi)

    return np.array([
        [ c, -s, 0.0, 0.0],
        [ s,  c, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def jacobian_dot(psi, r):

    c = np.cos(psi)
    s = np.sin(psi)

    return r * np.array([
        [-s, -c, 0.0, 0.0],
        [ c, -s, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
    ])


def sat(v, vmax):
    return np.clip(v, -vmax, vmax)


def simulate(params):

    kp_x, kp_y, kp_z, kp_yaw, \
    ksp_x, ksp_y, ksp_z, ksp_yaw, \
    kd_x, kd_y, kd_z, kd_yaw, \
    ksd_x, ksd_y, ksd_z, ksd_yaw = params

    KP = np.diag([
        kp_x,
        kp_y,
        kp_z,
        kp_yaw
    ])

    KSP = np.diag([
        ksp_x,
        ksp_y,
        ksp_z,
        ksp_yaw
    ])

    KD = np.diag([
        kd_x,
        kd_y,
        kd_z,
        kd_yaw
    ])

    KSD = np.diag([
        ksd_x,
        ksd_y,
        ksd_z,
        ksd_yaw
    ])


    eta = np.zeros(4)
    nu = np.zeros(4)

    Xdot_ref_prev = np.zeros(4)


    eta_hist = []
    ref_hist = []
    u_hist = []
    nu_hist = []
    time_hist = []


    for k in range(N):

        t = k * Ts

        idx = min(
            int(t / TIME_PER_POINT),
            len(WAYPOINTS)-1
        )

        eta_d = WAYPOINTS[idx]

        nu_d_ref = np.zeros(4)
        alpha_d = np.zeros(4)

        psi = eta[3]
        r = nu[3]


        J = jacobian(psi)

        Jdot = jacobian_dot(psi, r)

        F1_i = J @ F1

        F2_eff = J @ F2 - Jdot

        F1_inv = np.linalg.inv(F1_i)


        X_dot = J @ nu


        X_tilde = eta_d - eta

        X_tilde[3] = wrap_angle(X_tilde[3])

        X_dot_ref = (
            nu_d_ref
            + KSP @ np.tanh(KP @ X_tilde)
        )


        X_dot_tilde = X_dot_ref - X_dot

        X_ddot_ref = (
            X_dot_ref - Xdot_ref_prev
        ) / Ts

        Xdot_ref_prev = X_dot_ref.copy()

        Ud = F1_inv @ (
            alpha_d
            + X_ddot_ref
            + KSD @ np.tanh(KD @ X_dot_tilde)
            + F2_eff @ nu
        )

        U_body = sat(Ud, U_MAX)


        nu_dot = (
            F1 @ U_body
            - F2 @ nu
        )

        nu = nu + Ts * nu_dot


        eta_dot = J @ nu

        eta = eta + Ts * eta_dot

        eta[3] = wrap_angle(eta[3])


        eta_hist.append(eta.copy())
        ref_hist.append(eta_d.copy())
        u_hist.append(U_body.copy())
        nu_hist.append(nu.copy())
        time_hist.append(t)

    return (
        np.array(time_hist),
        np.array(eta_hist),
        np.array(ref_hist),
        np.array(u_hist),
        np.array(nu_hist)
    )


initial_params = [

    1.0, 1.0, 1.0, 1.0,

    0.8, 0.8, 0.5, 0.8,

    4.0, 4.0, 3.0, 3.0,

    0.9, 0.9, 0.35, 0.30
]


fig = plt.figure(figsize=(20, 12))

ax3d = fig.add_subplot(231, projection='3d')
ax_xy = fig.add_subplot(232)
ax_z = fig.add_subplot(233)

ax_u = fig.add_subplot(234)
ax_vel = fig.add_subplot(235)
ax_yaw = fig.add_subplot(236)

plt.subplots_adjust(
    left=0.28,
    right=0.98,
    top=0.97,
    bottom=0.04,
    hspace=0.3
)


sliders = []

labels = [

    "Kp_x",
    "Kp_y",
    "Kp_z",
    "Kp_yaw",

    "Ksp_x",
    "Ksp_y",
    "Ksp_z",
    "Ksp_yaw",

    "Kd_x",
    "Kd_y",
    "Kd_z",
    "Kd_yaw",

    "Ksd_x",
    "Ksd_y",
    "Ksd_z",
    "Ksd_yaw",
]

for i in range(16):

    ax = plt.axes([
        0.03,
        0.93 - i * 0.055,
        0.18,
        0.03
    ])

    s = Slider(
        ax=ax,
        label=labels[i],
        valmin=0.0,
        valmax=10.0,
        valinit=initial_params[i]
    )

    sliders.append(s)


def update(val):

    params = [s.val for s in sliders]

    (
        time_hist,
        eta_hist,
        ref_hist,
        u_hist,
        nu_hist
    ) = simulate(params)

    ax3d.cla()
    ax_xy.cla()
    ax_z.cla()

    ax_u.cla()
    ax_vel.cla()
    ax_yaw.cla()


    ax3d.plot(
        eta_hist[:,0],
        eta_hist[:,1],
        eta_hist[:,2],
        linewidth=2
    )

    ax3d.scatter(
        WAYPOINTS[:,0],
        WAYPOINTS[:,1],
        WAYPOINTS[:,2],
        s=120
    )

    ax3d.set_xlim([-2.5, 2.5])
    ax3d.set_ylim([-2.5, 2.5])
    ax3d.set_zlim([-2.5, 2.5])

    ax3d.set_xlabel("X")
    ax3d.set_ylabel("Y")
    ax3d.set_zlabel("Z")

    ax3d.set_title("Trayectoria 3D")

    ax3d.grid()


    ax_xy.plot(
        time_hist,
        eta_hist[:,0],
        label='x'
    )

    ax_xy.plot(
        time_hist,
        eta_hist[:,1],
        label='y'
    )

    ax_xy.plot(
        time_hist,
        ref_hist[:,0],
        '--'
    )

    ax_xy.plot(
        time_hist,
        ref_hist[:,1],
        '--'
    )

    ax_xy.set_title("Posicion XY")

    ax_xy.set_xlabel("Tiempo [s]")

    ax_xy.legend()

    ax_xy.grid()


    ax_z.plot(
        time_hist,
        eta_hist[:,2],
        label='z'
    )

    ax_z.plot(
        time_hist,
        ref_hist[:,2],
        '--'
    )

    ax_z.set_title("Altura")

    ax_z.set_xlabel("Tiempo [s]")

    ax_z.legend()

    ax_z.grid()


    ax_u.plot(
        time_hist,
        u_hist[:,0],
        label='Ux'
    )

    ax_u.plot(
        time_hist,
        u_hist[:,1],
        label='Uy'
    )

    ax_u.plot(
        time_hist,
        u_hist[:,2],
        label='Uz'
    )

    ax_u.plot(
        time_hist,
        u_hist[:,3],
        label='Ur'
    )

    ax_u.set_title("Entradas de Control")

    ax_u.set_xlabel("Tiempo [s]")

    ax_u.legend()

    ax_u.grid()


    ax_vel.plot(
        time_hist,
        nu_hist[:,0],
        label='u'
    )

    ax_vel.plot(
        time_hist,
        nu_hist[:,1],
        label='v'
    )

    ax_vel.plot(
        time_hist,
        nu_hist[:,2],
        label='w'
    )

    ax_vel.plot(
        time_hist,
        nu_hist[:,3],
        label='r'
    )

    ax_vel.set_title("Velocidades Body")

    ax_vel.set_xlabel("Tiempo [s]")

    ax_vel.legend()

    ax_vel.grid()


    ax_yaw.plot(
        time_hist,
        eta_hist[:,3],
        label='yaw'
    )

    ax_yaw.plot(
        time_hist,
        ref_hist[:,3],
        '--',
        linewidth=2
    )

    ax_yaw.set_title("Yaw")

    ax_yaw.set_xlabel("Tiempo [s]")

    ax_yaw.legend()

    ax_yaw.grid()

    fig.canvas.draw_idle()


reset_ax = plt.axes([0.08, 0.01, 0.1, 0.04])

button = Button(
    reset_ax,
    'Reset'
)

def reset(event):

    for s in sliders:
        s.reset()

button.on_clicked(reset)


for s in sliders:
    s.on_changed(update)


update(None)

plt.show()