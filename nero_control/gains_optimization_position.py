#!/usr/bin/env python3
"""
Optimizador de ganancias para el controlador en cascada del drone Bebop.

Simula internamente la dinámica del simulador (mismo modelo que BebopWebotsFullSim)
y busca las 16 ganancias independientes de KP, KSP, KD, KSD (diagonales 4x4)
que minimizan el error de seguimiento en la secuencia de referencias del RefPublisher.

Velocidades máximas reales en body frame:
    vx_max = 0.9 m/s,  vy_max = 0.9 m/s,  vz_max = 0.8 m/s,  r_max = 0.8 rad/s

Restricciones de diseño (sin ifs, solo ajustando ganancias):
  - Altura z:
      KSP_z  ≤ vz_max = 0.8 m/s en inercial (vel. ref. máxima)
      KP_z ∈ [4, 8]  → tanh satura en ±3/KP_z ≈ ±0.50 m
      KD_z grande    → amortiguamiento fuerte cerca del setpoint
  - Yaw:
      KSP_yaw ≤ r_max = 0.8 rad/s
      KP_yaw ∈ [8, 14] → tanh satura en ±3/KP_yaw ≈ ±15°

Las 16 ganancias son los 4 diagonales de cada una de las 4 matrices:
    KP  = diag(kp_x,  kp_y,  kp_z,  kp_yaw)
    KSP = diag(ksp_x, ksp_y, ksp_z, ksp_yaw)
    KD  = diag(kd_x,  kd_y,  kd_z,  kd_yaw)
    KSD = diag(ksd_x, ksd_y, ksd_z, ksd_yaw)
"""

import numpy as np
from scipy.optimize import differential_evolution
import time

# ---------------------------------------------------------------------------
# Dinámica del simulador (mismos parámetros que BebopWebotsFullSim)
# ---------------------------------------------------------------------------
_F1 = np.diag([0.988324, 0.986558, 0.802580, 0.853101])
_F2 = np.diag([0.018878, 0.025773, 0.122009, 0.122507])

DYN_DT  = 0.064        # 64 ms  (DYN_TIMESTEP_MS del sim)
CTRL_DT = 1.0 / 15.0  # ~66.7 ms (RATE_HZ del controlador)

# Matrices del modelo discreto en body frame (cascade_controller.py)
CTRL_F1 = np.array([
    [ 0.988324,  0.014589,  0.000000, -0.009546],
    [-0.019977,  0.986558,  0.000000,  0.005055],
    [ 0.000000,  0.000000,  0.802580,  0.000000],
    [ 0.000000,  0.000000,  0.000000,  0.853101],
])
CTRL_F2 = np.array([
    [ 0.018878, -0.002861,  0.000000,  0.003410],
    [-0.002938,  0.025773,  0.000000, -0.008888],
    [ 0.000000,  0.000000,  0.122009,  0.000000],
    [ 0.000000,  0.000000,  0.000000,  0.122507],
])

# Velocidades máximas en body frame [m/s, m/s, m/s, rad/s]
NU_MAX = np.array([0.9, 0.9, 0.8, 0.8])

# U_MAX del controlador (saturación del comando normalizado)
U_MAX = np.ones(4)

# ---------------------------------------------------------------------------
# Referencia (igual que RefPublisher, posición inicial [0, 0, 1.5, 0])
# ---------------------------------------------------------------------------
HOLD_TIME    = 10.0    # segundos por pose
L            = 1.5
SIM_DURATION = 60.0   # 1 minuto

POINTS = np.array([
    [ L/2,  L/2, 1.2],
    [-L/2, -L/2, 1.5],
    [-L/2, -L/2, 1.8],
    [ L/2,  L/2, 1.3],
])
YAWS = np.deg2rad([45.0, 45.0, -45.0, -45.0])

# ---------------------------------------------------------------------------
# Utilidades
# ---------------------------------------------------------------------------
def wrap_angle(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi

def J(psi):
    c, s = np.cos(psi), np.sin(psi)
    return np.array([
        [ c, -s, 0., 0.],
        [ s,  c, 0., 0.],
        [0., 0., 1., 0.],
        [0., 0., 0., 1.],
    ])

def J_dot(psi, r):
    c, s = np.cos(psi), np.sin(psi)
    return r * np.array([
        [-s, -c, 0., 0.],
        [ c, -s, 0., 0.],
        [0., 0., 0., 0.],
        [0., 0., 0., 0.],
    ])

def get_ref_at(t):
    """Devuelve (eta_d, nu_d=0, alpha_d=0) para el tiempo t, ciclo infinito."""
    idx     = int(t / HOLD_TIME) % len(POINTS)
    eta_d   = np.array([POINTS[idx, 0], POINTS[idx, 1], POINTS[idx, 2], YAWS[idx]])
    nu_d    = np.zeros(4)
    alpha_d = np.zeros(4)
    return eta_d, nu_d, alpha_d

# ---------------------------------------------------------------------------
# Simulación completa
# ---------------------------------------------------------------------------
def simulate(KP, KSP, KD, KSD):
    """
    Simula SIM_DURATION segundos desde eta=[0,0,1.5,0], nu=[0,0,0,0].
    Devuelve array de errores de posición (N, 4) en cada paso de control.
    """
    eta = np.array([0., 0., 1.5, 0.])
    nu  = np.zeros(4)
    X_dot_ref_prev = np.zeros(4)
    errors = []
    t = 0.0
    sim_steps = max(1, round(CTRL_DT / DYN_DT))  # pasos de dinámica por ciclo ctrl

    while t < SIM_DURATION:
        eta_d, nu_d, alpha_d = get_ref_at(t)

        psi  = eta[3]
        r    = nu[3]

        Jmat   = J(psi)
        Jdot   = J_dot(psi, r)
        F1_eff = Jmat @ CTRL_F1
        F2_eff = Jmat @ CTRL_F2 - Jdot
        F1_inv = np.linalg.inv(F1_eff)

        X_dot = Jmat @ nu

        # Error de posición (lazo externo)
        X_tilde    = eta_d - eta
        X_tilde[3] = wrap_angle(X_tilde[3])

        # Velocidad de referencia: KSP limita la vel. máx. que puede pedir el lazo externo
        X_dot_ref   = nu_d + KSP @ np.tanh(KP @ X_tilde)

        # Error de velocidad (lazo interno)
        X_dot_tilde = X_dot_ref - X_dot
        X_ddot_ref  = (X_dot_ref - X_dot_ref_prev) / CTRL_DT
        X_dot_ref_prev = X_dot_ref.copy()

        # Ley de control
        Ud = F1_inv @ (
            alpha_d
            + X_ddot_ref
            + KSD @ np.tanh(KD @ X_dot_tilde)
            + F2_eff @ nu
        )
        U_body = np.clip(Ud, -U_MAX, U_MAX)

        errors.append(X_tilde.copy())

        # Integrar dinámica con sub-pasos
        for _ in range(sim_steps):
            nu_dot = _F1 @ U_body - _F2 @ nu
            nu     = nu + DYN_DT * nu_dot
            # Límites físicos body frame
            nu = np.clip(nu, -NU_MAX, NU_MAX)

            eta_dot  = J(eta[3]) @ nu
            eta      = eta + DYN_DT * eta_dot
            eta[3]   = wrap_angle(eta[3])
            if eta[2] < 0.05:
                eta[2] = 0.05
                nu[2]  = 0.0

        t += CTRL_DT

    return np.array(errors)   # (N, 4)

# ---------------------------------------------------------------------------
# Función de coste
# ---------------------------------------------------------------------------
# Parámetros (16 en total):
#  índices  0..3  → kp_x,  kp_y,  kp_z,  kp_yaw
#  índices  4..7  → ksp_x, ksp_y, ksp_z, ksp_yaw
#  índices  8..11 → kd_x,  kd_y,  kd_z,  kd_yaw
#  índices 12..15 → ksd_x, ksd_y, ksd_z, ksd_yaw

def unpack(params):
    KP  = np.diag(params[0:4])
    KSP = np.diag(params[4:8])
    KD  = np.diag(params[8:12])
    KSD = np.diag(params[12:16])
    return KP, KSP, KD, KSD

def cost(params):
    KP, KSP, KD, KSD = unpack(params)

    try:
        errors = simulate(KP, KSP, KD, KSD)
    except Exception:
        return 1e9

    if not np.all(np.isfinite(errors)):
        return 1e9

    N = len(errors)

    # Pesos por canal: z y yaw tienen mayor peso (errores de medición en práctica)
    w = np.array([1.0, 1.0, 2.5, 2.5])

    # ISE ponderado
    ise = np.mean((errors ** 2) * w)

    # Penalizar transitorio (primer cuarto de la simulación pesa más)
    n_trans = N // 4
    transient_penalty = np.mean((errors[:n_trans] ** 2) * w) * 0.4

    # Penalizar error en estado estacionario (último cuarto)
    ss_penalty = np.mean((errors[-n_trans:] ** 2) * w) * 1.5

    return ise + transient_penalty + ss_penalty

# ---------------------------------------------------------------------------
# Bounds (min, max) para las 16 ganancias
#
# Restricciones de diseño codificadas en los bounds:
#
#  KP_z   ∈ [4.0, 8.0]   → tanh≈1 cuando |e_z|  > 3/KP_z ≈ 0.50 m   ✓
#  KP_yaw ∈ [8.0, 14.0]  → tanh≈1 cuando |e_ψ|  > 3/KP_ψ ≈ 15°     ✓
#
#  KSP_x  ≤ 0.9  (vx_max body)
#  KSP_y  ≤ 0.9  (vy_max body)
#  KSP_z  ≤ 0.8  (vz_max body) → velocidad máxima en z = KSP_z       ✓
#  KSP_yaw≤ 0.8  (r_max  body) → velocidad máxima en yaw = KSP_yaw   ✓
# ---------------------------------------------------------------------------
BOUNDS = [
    # KP diagonal [kp_x, kp_y, kp_z, kp_yaw]
    (0.3,  4.0),    # kp_x
    (0.3,  4.0),    # kp_y
    (4.0,  8.0),    # kp_z   ← restricción zona ±50 cm
    (8.0, 14.0),    # kp_yaw ← restricción zona ±15°

    # KSP diagonal [ksp_x, ksp_y, ksp_z, ksp_yaw]
    (0.2,  0.9),    # ksp_x  ← ≤ vx_max
    (0.2,  0.9),    # ksp_y  ← ≤ vy_max
    (0.1,  0.8),    # ksp_z  ← ≤ vz_max, es la vel. máx. en z
    (0.1,  0.8),    # ksp_yaw← ≤ r_max,  es la vel. máx. en yaw

    # KD diagonal [kd_x, kd_y, kd_z, kd_yaw]
    (1.0, 10.0),    # kd_x
    (1.0, 10.0),    # kd_y
    (0.5,  5.0),    # kd_z
    (0.3,  3.0),    # kd_yaw

    # KSD diagonal [ksd_x, ksd_y, ksd_z, ksd_yaw]
    (0.2,  2.0),    # ksd_x
    (0.2,  2.0),    # ksd_y
    (0.1,  1.0),    # ksd_z
    (0.1,  0.8),    # ksd_yaw
]

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    print("=" * 65)
    print("Optimizador de ganancias — Drone Bebop Cascade Controller")
    print("  16 ganancias independientes (diagonales de KP, KSP, KD, KSD)")
    print("=" * 65)
    print(f"Duración de simulación : {SIM_DURATION} s")
    print(f"Vel. máx. body frame   : vx={NU_MAX[0]}, vy={NU_MAX[1]}, "
          f"vz={NU_MAX[2]}, r={NU_MAX[3]} rad/s")
    kp_z_mid   = (BOUNDS[2][0] + BOUNDS[2][1]) / 2
    kp_yaw_mid = (BOUNDS[3][0] + BOUNDS[3][1]) / 2
    print(f"Objetivo z             : saturación tanh en ±{3/kp_z_mid:.2f} m (medio rango KP_z)")
    print(f"Objetivo yaw           : saturación tanh en ±{np.degrees(3/kp_yaw_mid):.1f}° (medio rango KP_yaw)")
    print()

    t0 = time.time()

    result = differential_evolution(
        cost,
        BOUNDS,
        strategy='best1bin',
        maxiter=10,
        popsize=18,        # 18 × 16 params = 288 individuos por generación
        tol=1e-6,
        mutation=(0.5, 1.2),
        recombination=0.85,
        seed=42,
        polish=True,
        disp=True,
        workers=-1,        # usa todos los núcleos disponibles
        updating='deferred',
    )

    elapsed = time.time() - t0
    print(f"\nOptimización finalizada en {elapsed:.1f} s")
    print(f"Coste final: {result.fun:.6f}")

    KP, KSP, KD, KSD = unpack(result.x)

    def fmt_mat(name, mat):
        d = np.diag(mat)
        return (f"    {name} = np.diag([{d[0]:.6f}, {d[1]:.6f}, "
                f"{d[2]:.6f}, {d[3]:.6f}])")

    lines = [
        "─" * 65,
        "GANANCIAS ÓPTIMAS (para pegar en cascade_controller.py)",
        "─" * 65,
        fmt_mat("KP ", KP),
        fmt_mat("KSP", KSP),
        fmt_mat("KD ", KD),
        fmt_mat("KSD", KSD),
    ]

    print()
    for l in lines:
        print(l)

    # Análisis
    kp_z   = np.diag(KP)[2]
    kp_yaw = np.diag(KP)[3]
    ksp_z  = np.diag(KSP)[2]
    ksp_yaw= np.diag(KSP)[3]

    print()
    print("─" * 65)
    print("ANÁLISIS DE DISEÑO")
    print("─" * 65)
    print(f"  KP_z   = {kp_z:.4f}  → zona saturación z:   ±{3/kp_z:.3f} m  (objetivo ±0.50 m)")
    print(f"  KP_yaw = {kp_yaw:.4f}  → zona saturación yaw: ±{np.degrees(3/kp_yaw):.1f}°  (objetivo ±15°)")
    print(f"  KSP_z  = {ksp_z:.4f}  → vel. máx. referencia z    (límite físico {NU_MAX[2]} m/s)")
    print(f"  KSP_yaw= {ksp_yaw:.4f}  → vel. máx. referencia yaw  (límite físico {NU_MAX[3]} rad/s)")

    errors = simulate(KP, KSP, KD, KSD)
    rms = np.sqrt(np.mean(errors ** 2, axis=0))
    print()
    print("  RMS error (simulación completa):")
    print(f"    x:   {rms[0]*100:.2f} cm")
    print(f"    y:   {rms[1]*100:.2f} cm")
    print(f"    z:   {rms[2]*100:.2f} cm")
    print(f"    yaw: {np.degrees(rms[3]):.3f}°")

    # Guardar resultado
    out_path = "optimal_gains.txt"
    with open(out_path, "w") as f:
        f.write("# Ganancias óptimas para cascade_controller.py\n")
        f.write(f"# Coste ISE total: {result.fun:.6f}\n")
        f.write(f"# KP_z={np.diag(KP)[2]:.4f}  → zona sat z:  ±{3/np.diag(KP)[2]:.3f} m\n")
        f.write(f"# KP_yaw={np.diag(KP)[3]:.4f} → zona sat yaw: ±{np.degrees(3/np.diag(KP)[3]):.1f}°\n")
        f.write(f"# KSP_z={np.diag(KSP)[2]:.4f}  → vel.max z:   {np.diag(KSP)[2]:.3f} m/s\n")
        f.write(f"# KSP_yaw={np.diag(KSP)[3]:.4f} → vel.max yaw: {np.diag(KSP)[3]:.3f} rad/s\n\n")
        for l in lines[3:]:
            f.write(l.strip() + "\n")

    print(f"\nResultado guardado en: {out_path}")


if __name__ == "__main__":
    main()