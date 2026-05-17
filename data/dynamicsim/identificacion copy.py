#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from scipy.io import loadmat
from scipy.signal import savgol_filter

mat_file = "manual_log_20260516_201622.mat"
data = loadmat(mat_file)
hz = float(data["hz"].squeeze())
dt = 1.0 / hz

DELAY = 2
MIN_INPUT = 0.0

u_full = data["u"]
vx_world_full = data["vx_b"].squeeze()
vy_world_full = data["vy_b"].squeeze()
vz_full = data["vz_b"].squeeze()
psi_full = data["psi"].squeeze()
r_full = data["vpsi"].squeeze()

x_real_full = data["x_i"].squeeze()
y_real_full = data["y_i"].squeeze()
z_real_full = data["z_i"].squeeze()

if DELAY > 0:
    u = u_full[:-DELAY, :]
    vx_world = vx_world_full[DELAY:]
    vy_world = vy_world_full[DELAY:]
    vz_raw = vz_full[DELAY:]
    psi = psi_full[DELAY:]
    r_raw = r_full[DELAY:]
    
    x_real = x_real_full[DELAY:]
    y_real = y_real_full[DELAY:]
    z_real = z_real_full[DELAY:]
else:
    u = u_full
    vx_world = vx_world_full
    vy_world = vy_world_full
    vz_raw = vz_full
    psi = psi_full
    r_raw = r_full
    
    x_real = x_real_full
    y_real = y_real_full
    z_real = z_real_full

ux = u[:,0]
uy = u[:,1]
uz = u[:,2]
upsi = u[:,3]

vx_raw = np.cos(psi)*vx_world + np.sin(psi)*vy_world
vy_raw = -np.sin(psi)*vx_world + np.cos(psi)*vy_world

# Filtros para XY (Window=3, Poly=1)
vx_filt = savgol_filter(vx_raw, 3, 1)
vy_filt = savgol_filter(vy_raw, 3, 1)
# Filtros especificos para Z y YAW obtenidos antes
vz_filt = savgol_filter(vz_raw, 11, 2)
r_filt = savgol_filter(r_raw, 9, 3)

def identify_coupled(vx, vy, r, ux, uy, min_input):
    y_x = vx[1:]
    Phi_x = np.column_stack([vx[:-1], vy[:-1], r[:-1], ux[:-1]])
    
    y_y = vy[1:]
    Phi_y = np.column_stack([vx[:-1], vy[:-1], r[:-1], uy[:-1]])
    
    mask = (np.abs(ux[:-1]) > min_input) | (np.abs(uy[:-1]) > min_input)
    
    y_x = y_x[mask]
    Phi_x = Phi_x[mask]
    y_y = y_y[mask]
    Phi_y = Phi_y[mask]
    
    theta_x, *_ = np.linalg.lstsq(Phi_x, y_x, rcond=None)
    theta_y, *_ = np.linalg.lstsq(Phi_y, y_y, rcond=None)
    return theta_x, theta_y

def identify_decoupled(v, u, min_input):
    y = v[1:]
    vk = v[:-1]
    uk = u[:-1]
    mask = np.abs(uk) > min_input
    y = y[mask]
    vk = vk[mask]
    uk = uk[mask]
    Phi = np.column_stack([vk, uk])
    theta, *_ = np.linalg.lstsq(Phi, y, rcond=None)
    return theta[0], theta[1]

# Identificacion Discreta
theta_x, theta_y = identify_coupled(vx_filt, vy_filt, r_filt, ux, uy, MIN_INPUT)
alpha_z, beta_z = identify_decoupled(vz_filt, uz, MIN_INPUT)
alpha_r, beta_r = identify_decoupled(r_filt, upsi, MIN_INPUT)

Ad = np.array([
    [theta_x[0], theta_x[1], 0.0, theta_x[2]],
    [theta_y[0], theta_y[1], 0.0, theta_y[2]],
    [0.0,        0.0,        alpha_z, 0.0],
    [0.0,        0.0,        0.0,     alpha_r]
])

Bd = np.array([
    [theta_x[3], 0.0,        0.0,    0.0],
    [0.0,        theta_y[3], 0.0,    0.0],
    [0.0,        0.0,        beta_z, 0.0],
    [0.0,        0.0,        0.0,    beta_r]
])

print("\n================================================")
print("MATRIZ A DISCRETA (Ad)")
print("================================================")
np.set_printoptions(formatter={'float': lambda x: "{0:0.6f}".format(x)})
print(Ad)

print("\n================================================")
print("MATRIZ B DISCRETA (Bd)")
print("================================================")
print(Bd)

# Simulacion global
N = len(ux)
X_sim = np.zeros((4, N))
X_sim[0,0] = vx_filt[0]
X_sim[1,0] = vy_filt[0]
X_sim[2,0] = vz_filt[0]
X_sim[3,0] = r_filt[0]

U = np.vstack([ux, uy, uz, upsi])

for k in range(N-1):
    X_sim[:, k+1] = Ad @ X_sim[:, k] + Bd @ U[:, k]

vx_hat = X_sim[0, :]
vy_hat = X_sim[1, :]
vz_hat = X_sim[2, :]
r_hat  = X_sim[3, :]

def fit(y, yhat):
    var_y = np.linalg.norm(y - np.mean(y))
    if var_y == 0: return 0.0
    return 100 * (1 - np.linalg.norm(y - yhat) / var_y)

print("\n================================================")
print("VELOCITY FIT")
print("================================================")
print(f"FIT VX   : {fit(vx_filt, vx_hat):.2f}%")
print(f"FIT VY   : {fit(vy_filt, vy_hat):.2f}%")
print(f"FIT VZ   : {fit(vz_filt, vz_hat):.2f}%")
print(f"FIT YAW  : {fit(r_filt, r_hat):.2f}%")

# Simulacion de posiciones
x_sim = np.zeros(N)
y_sim = np.zeros(N)
z_sim = np.zeros(N)
psi_sim = np.zeros(N)

x_sim[0] = x_real[0]
y_sim[0] = y_real[0]
z_sim[0] = z_real[0]
psi_sim[0] = psi[0]

def kinematics(state, v_body):
    psi = state[3]
    vx, vy, vz, r = v_body
    xdot = np.cos(psi)*vx - np.sin(psi)*vy
    ydot = np.sin(psi)*vx + np.cos(psi)*vy
    zdot = vz
    psidot = r
    return np.array([xdot, ydot, zdot, psidot])

for k in range(N-1):
    state_k = np.array([x_sim[k], y_sim[k], z_sim[k], psi_sim[k]])
    
    v_k = np.array([vx_hat[k], vy_hat[k], vz_hat[k], r_hat[k]])
    v_kp1 = np.array([vx_hat[k+1], vy_hat[k+1], vz_hat[k+1], r_hat[k+1]])
    v_mid = 0.5 * (v_k + v_kp1)
    
    k1 = kinematics(state_k, v_k)
    k2 = kinematics(state_k + 0.5 * dt * k1, v_mid)
    k3 = kinematics(state_k + 0.5 * dt * k2, v_mid)
    k4 = kinematics(state_k + dt * k3, v_kp1)
    
    state_kp1 = state_k + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
    
    x_sim[k+1] = state_kp1[0]
    y_sim[k+1] = state_kp1[1]
    z_sim[k+1] = state_kp1[2]
    psi_sim[k+1] = state_kp1[3]

print("\n================================================")
print("POSITION FIT")
print("================================================")
print(f"FIT X POSITION   : {fit(x_real, x_sim):.2f}%")
print(f"FIT Y POSITION   : {fit(y_real, y_sim):.2f}%")
print(f"FIT Z POSITION   : {fit(z_real, z_sim):.2f}%")
print(f"FIT PSI POSITION : {fit(psi, psi_sim):.2f}%")

# Plots
t = np.arange(N) * dt

fig, axs = plt.subplots(2, 2, figsize=(14, 8))
fig.suptitle("Ajuste de Velocidades (Modelo Discreto Acoplado en XY)")

axs[0,0].plot(t, vx_raw, label='Real (medido)', color='lightblue', alpha=0.5)
axs[0,0].plot(t, vx_filt, label='Real (filtrado)', color='blue', alpha=0.8)
axs[0,0].plot(t, vx_hat, '--', label='Modelo', color='red')
axs[0,0].set_title("VX (BODY)")
axs[0,0].legend(); axs[0,0].grid()

axs[0,1].plot(t, vy_raw, label='Real (medido)', color='lightblue', alpha=0.5)
axs[0,1].plot(t, vy_filt, label='Real (filtrado)', color='blue', alpha=0.8)
axs[0,1].plot(t, vy_hat, '--', label='Modelo', color='red')
axs[0,1].set_title("VY (BODY)")
axs[0,1].legend(); axs[0,1].grid()

axs[1,0].plot(t, vz_raw, label='Real (medido)', color='lightblue', alpha=0.5)
axs[1,0].plot(t, vz_filt, label='Real (filtrado)', color='blue', alpha=0.8)
axs[1,0].plot(t, vz_hat, '--', label='Modelo', color='red')
axs[1,0].set_title("VZ")
axs[1,0].legend(); axs[1,0].grid()

axs[1,1].plot(t, r_raw, label='Real (medido)', color='lightblue', alpha=0.5)
axs[1,1].plot(t, r_filt, label='Real (filtrado)', color='blue', alpha=0.8)
axs[1,1].plot(t, r_hat, '--', label='Modelo', color='red')
axs[1,1].set_title("YAW RATE")
axs[1,1].legend(); axs[1,1].grid()

plt.tight_layout()

fig2, axs2 = plt.subplots(2, 2, figsize=(14, 8))
fig2.suptitle("Ajuste de Posiciones / Trayectoria")

axs2[0,0].plot(t, x_real, label='Real', color='lightgray')
axs2[0,0].plot(t, x_sim, '--', label='Simulado', color='blue')
axs2[0,0].set_title("Posicion X")
axs2[0,0].legend(); axs2[0,0].grid()

axs2[0,1].plot(t, y_real, label='Real', color='lightgray')
axs2[0,1].plot(t, y_sim, '--', label='Simulado', color='blue')
axs2[0,1].set_title("Posicion Y")
axs2[0,1].legend(); axs2[0,1].grid()

axs2[1,0].plot(t, z_real, label='Real', color='lightgray')
axs2[1,0].plot(t, z_sim, '--', label='Simulado', color='blue')
axs2[1,0].set_title("Posicion Z")
axs2[1,0].legend(); axs2[1,0].grid()

axs2[1,1].plot(t, psi, label='Real', color='lightgray')
axs2[1,1].plot(t, psi_sim, '--', label='Simulado', color='blue')
axs2[1,1].set_title("Posicion YAW")
axs2[1,1].legend(); axs2[1,1].grid()

plt.tight_layout()
plt.show()