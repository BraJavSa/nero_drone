#!/usr/bin/env python3

"""
Identificación dinámica 4DOF para Bebop
+ simulación completa de trayectoria.

Suposiciones:
- cmd_vel está en BODY
- velocidades del EKF vienen en WORLD
- se transforman WORLD -> BODY usando yaw

Modelo dinámico discreto:

vx[k+1] = alpha_x*vx[k] + beta_x*ux[k]
vy[k+1] = alpha_y*vy[k] + beta_y*uy[k]
vz[k+1] = alpha_z*vz[k] + beta_z*uz[k]
r[k+1]  = alpha_r*r[k] + beta_r*upsi[k]

Modelo cinemático:

xdot = cos(psi)*vx - sin(psi)*vy
ydot = sin(psi)*vx + cos(psi)*vy
zdot = vz
psidot = r
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.io import loadmat
from scipy.signal import savgol_filter

mat_file = "manual_log_20260516_201622.mat"
USE_FILTER = True
OPTIMIZE_FILTER = True  # True para buscar los mejores parametros
WINDOW = 5
POLY = 3
MIN_INPUT = 0.02
DELAY = 2

data = loadmat(mat_file)
hz = float(data["hz"].squeeze())
dt = 1.0 / hz

print("\n================================================")
print("DATA")
print("================================================")
print(f"Frecuencia: {hz} Hz")
print(f"dt: {dt}")

# Alineamos los datos: la accion u[k] afecta la velocidad en k+DELAY
u = data["u"][:-DELAY, :] if DELAY > 0 else data["u"]
ux = u[:,0]
uy = u[:,1]
uz = u[:,2]
upsi = u[:,3]

x_real = data["x_i"].squeeze()[DELAY:]
y_real = data["y_i"].squeeze()[DELAY:]
z_real = data["z_i"].squeeze()[DELAY:]
psi_real = data["psi"].squeeze()[DELAY:]

vx_world = data["vx_b"].squeeze()[DELAY:]
vy_world = data["vy_b"].squeeze()[DELAY:]
vz = data["vz_b"].squeeze()[DELAY:]
r = data["vpsi"].squeeze()[DELAY:]
psi = data["psi"].squeeze()[DELAY:]

vx = np.cos(psi)*vx_world + np.sin(psi)*vy_world
vy = -np.sin(psi)*vx_world + np.cos(psi)*vy_world

print("\nVelocidades transformadas WORLD -> BODY")

if USE_FILTER and OPTIMIZE_FILTER:
    print("\nOptimizando filtros independientes por eje basado en RMSE de posiciones...")
    
    combinations = []
    for w in [3, 5, 7, 9, 11, 15, 21, 31]:
        for p in [1, 2, 3, 4]:
            if p < w: combinations.append((w, p))
            
    def q_id(v, u):
        y = v[1:]
        vk = v[:-1]
        uk = u[:-1]
        m = np.abs(uk) > MIN_INPUT
        y = y[m]; vk = vk[m]; uk = uk[m]
        try:
            th, *_ = np.linalg.lstsq(np.column_stack([vk, uk]), y, rcond=None)
            return th[0], th[1]
        except:
            return 0, 0

    N_sim = len(ux)
    best_z = (11, 2)
    best_yaw = (9, 3)
    r_f = savgol_filter(r, best_yaw[0], best_yaw[1])
    apsid, bpsid = q_id(r_f, upsi)
    best_psi_s = np.zeros(N_sim); r_s = np.zeros(N_sim)
    best_psi_s[0]=psi_real[0]; r_s[0]=r_f[0]
    for k in range(N_sim-1):
        r_s[k+1] = apsid*r_s[k] + bpsid*upsi[k]
        best_psi_s[k+1] = best_psi_s[k] + dt*r_s[k]
            
    # 3. Optimizar X e Y
    # Expandimos las combinaciones para encontrar mejores valores
    combinations_xy = []
    for w in [3, 5, 7, 9, 11, 15, 21, 31, 41, 51, 61, 71, 81, 91]:
        for p in [1, 2, 3, 4]:
            if p < w: combinations_xy.append((w, p))

    # Pre-calcular secuencias vx_sim y vy_sim para evitar recalcular en cada iteracion
    vx_sims = {}
    for w, p in combinations_xy:
        vx_f = savgol_filter(vx, w, p)
        axd, bxd = q_id(vx_f, ux)
        vx_s = np.zeros(N_sim); vx_s[0] = vx_f[0]
        for k in range(N_sim-1):
            vx_s[k+1] = axd*vx_s[k] + bxd*ux[k]
        vx_sims[(w,p)] = vx_s
        
    vy_sims = {}
    for w, p in combinations_xy:
        vy_f = savgol_filter(vy, w, p)
        ayd, byd = q_id(vy_f, uy)
        vy_s = np.zeros(N_sim); vy_s[0] = vy_f[0]
        for k in range(N_sim-1):
            vy_s[k+1] = ayd*vy_s[k] + byd*uy[k]
        vy_sims[(w,p)] = vy_s
        
    best_rmse_xy = float('inf')
    best_x = (5, 3); best_y = (5, 3)
    
    cos_psi = np.cos(best_psi_s)
    sin_psi = np.sin(best_psi_s)
    
    for wx, px in combinations_xy:
        vx_s = vx_sims[(wx, px)]
        for wy, py in combinations_xy:
            vy_s = vy_sims[(wy, py)]
            
            xdot = cos_psi * vx_s - sin_psi * vy_s
            ydot = sin_psi * vx_s + cos_psi * vy_s
            
            x_s = x_real[0] + np.concatenate(([0], np.cumsum(xdot[:-1] * dt)))
            y_s = y_real[0] + np.concatenate(([0], np.cumsum(ydot[:-1] * dt)))
            
            rmse = np.sqrt(np.mean((x_real - x_s)**2)) + np.sqrt(np.mean((y_real - y_s)**2))
            
            if rmse < best_rmse_xy:
                best_rmse_xy = rmse
                best_x = (wx, px); best_y = (wy, py)

    print(f"Filtro Z (Fijo)  : WINDOW = {best_z[0]:2d}, POLY = {best_z[1]:2d}")
    print(f"Filtro YAW (Fijo): WINDOW = {best_yaw[0]:2d}, POLY = {best_yaw[1]:2d}")
    print(f"Mejor filtro X   : WINDOW = {best_x[0]:2d}, POLY = {best_x[1]:2d}")
    print(f"Mejor filtro Y   : WINDOW = {best_y[0]:2d}, POLY = {best_y[1]:2d} (RMSE X+Y: {best_rmse_xy:.4f})")
    
    WINDOW_X, POLY_X = best_x
    WINDOW_Y, POLY_Y = best_y
    WINDOW_Z, POLY_Z = best_z
    WINDOW_YAW, POLY_YAW = best_yaw

else:
    WINDOW_X = WINDOW_Y = WINDOW_Z = WINDOW_YAW = WINDOW
    POLY_X = POLY_Y = POLY_Z = POLY_YAW = POLY

if USE_FILTER:
    vx = savgol_filter(vx, WINDOW_X, POLY_X)
    vy = savgol_filter(vy, WINDOW_Y, POLY_Y)
    vz = savgol_filter(vz, WINDOW_Z, POLY_Z)
    r = savgol_filter(r, WINDOW_YAW, POLY_YAW)
    print("\nFiltros independientes Savitzky-Golay aplicados")

def identify_arx(v, u, name, verbose=True):
    y = v[1:]
    vk = v[:-1]
    uk = u[:-1]
    
    mask = np.abs(uk) > MIN_INPUT
    y = y[mask]
    vk = vk[mask]
    uk = uk[mask]
    
    if verbose:
        print(f"\n{name}")
        print(f"Muestras usadas: {len(y)}")
    
    Phi = np.column_stack([vk, uk])
    theta, *_ = np.linalg.lstsq(Phi, y, rcond=None)
    
    alpha = theta[0]
    beta = theta[1]
    
    a = (1.0 - alpha)/dt
    b = beta/dt
    
    if verbose:
        print(f"alpha = {alpha:.6f}")
        print(f"beta  = {beta:.6f}")
        print(f"a = {a:.6f}")
        print(f"b = {b:.6f}")
    
    return alpha, beta, a, b

axd, bxd, ax, bx = identify_arx(vx, ux, "X")
ayd, byd, ay, by = identify_arx(vy, uy, "Y")
azd, bzd, az, bz = identify_arx(vz, uz, "Z", verbose=False)
apsid, bpsid, apsi, bpsi = identify_arx(r, upsi, "YAW", verbose=False)

def simulate(alpha, beta, u, v0):
    N = len(u)
    y = np.zeros(N)
    y[0] = v0
    for k in range(N-1):
        y[k+1] = alpha*y[k] + beta*u[k]
    return y

vx_hat = simulate(axd, bxd, ux, vx[0])
vy_hat = simulate(ayd, byd, uy, vy[0])
vz_hat = simulate(azd, bzd, uz, vz[0])
r_hat = simulate(apsid, bpsid, upsi, r[0])

def fit(y, yhat):
    return 100 * (1 - np.linalg.norm(y - yhat) / np.linalg.norm(y - np.mean(y)))

fit_x = fit(vx, vx_hat)
fit_y = fit(vy, vy_hat)
fit_z = fit(vz, vz_hat)
fit_r = fit(r, r_hat)

print("\n================================================")
print("VELOCITY FIT")
print("================================================")
print(f"FIT VX   : {fit_x:.2f}%")
print(f"FIT VY   : {fit_y:.2f}%")
print(f"FIT VZ   : {fit_z:.2f}%")
print(f"FIT YAW  : {fit_r:.2f}%")

N = len(ux)
x_sim = np.zeros(N)
y_sim = np.zeros(N)
z_sim = np.zeros(N)
psi_sim = np.zeros(N)
vx_sim = np.zeros(N)
vy_sim = np.zeros(N)
vz_sim = np.zeros(N)
r_sim = np.zeros(N)

x_sim[0] = x_real[0]
y_sim[0] = y_real[0]
z_sim[0] = z_real[0]
psi_sim[0] = psi_real[0]
vx_sim[0] = vx[0]
vy_sim[0] = vy[0]
vz_sim[0] = vz[0]
r_sim[0] = r[0]

for k in range(N-1):
    vx_sim[k+1] = axd*vx_sim[k] + bxd*ux[k]
    vy_sim[k+1] = ayd*vy_sim[k] + byd*uy[k]
    vz_sim[k+1] = azd*vz_sim[k] + bzd*uz[k]
    r_sim[k+1] = apsid*r_sim[k] + bpsid*upsi[k]
    
    xdot = np.cos(psi_sim[k])*vx_sim[k] - np.sin(psi_sim[k])*vy_sim[k]
    ydot = np.sin(psi_sim[k])*vx_sim[k] + np.cos(psi_sim[k])*vy_sim[k]
    zdot = vz_sim[k]
    psidot = r_sim[k]
    
    x_sim[k+1] = x_sim[k] + dt*xdot
    y_sim[k+1] = y_sim[k] + dt*ydot
    z_sim[k+1] = z_sim[k] + dt*zdot
    psi_sim[k+1] = psi_sim[k] + dt*psidot

fit_x_pos = fit(x_real, x_sim)
fit_y_pos = fit(y_real, y_sim)
fit_z_pos = fit(z_real, z_sim)
fit_psi_pos = fit(psi_real, psi_sim)

print("\n================================================")
print("POSITION FIT")
print("================================================")
print(f"FIT X POSITION   : {fit_x_pos:.2f}%")
print(f"FIT Y POSITION   : {fit_y_pos:.2f}%")
print(f"FIT Z POSITION   : {fit_z_pos:.2f}%")
print(f"FIT PSI POSITION : {fit_psi_pos:.2f}%")

print("\n================================================")
print("CONTINUOUS MODEL")
print("================================================")
print("\nX")
print(f"dvx = -({ax:.6f})vx + ({bx:.6f})ux")
print("\nY")
print(f"dvy = -({ay:.6f})vy + ({by:.6f})uy")
print("\nZ")
print(f"dvz = -({az:.6f})vz + ({bz:.6f})uz")
print("\nYAW")
print(f"dr = -({apsi:.6f})r + ({bpsi:.6f})upsi")

A = np.array([
    [-ax,   0,    0,     0],
    [0,   -ay,    0,     0],
    [0,     0,  -az,     0],
    [0,     0,    0,  -apsi]
])

B = np.array([
    [bx,    0,    0,     0],
    [0,    by,    0,     0],
    [0,     0,   bz,     0],
    [0,     0,    0,  bpsi]
])

print("\n================================================")
print("MATRIZ A")
print("================================================")
print(A)

print("\n================================================")
print("MATRIZ B")
print("================================================")
print(B)

t = np.arange(N)*dt

def plot_channel(t, real, est, title, ylabel):
    plt.figure(figsize=(12,5))
    plt.plot(t, real, label="real")
    plt.plot(t, est, "--", label="modelo")
    plt.title(title)
    plt.xlabel("t [s]")
    plt.ylabel(ylabel)
    plt.grid()
    plt.legend()

plot_channel(t, vx, vx_hat, "VX BODY", "m/s")
plot_channel(t, vy, vy_hat, "VY BODY", "m/s")
# plot_channel(t, vz, vz_hat, "VZ", "m/s")
# plot_channel(t, r, r_hat, "YAW RATE", "rad/s")



plot_channel(t, x_real, x_sim, "POSICION X", "m")
plot_channel(t, y_real, y_sim, "POSICION Y", "m")
# plot_channel(t, z_real, z_sim, "POSICION Z", "m")
# plot_channel(t, psi_real, psi_sim, "YAW", "rad")



plt.show()