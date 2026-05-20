# Trains an MLP neural network to approximate the drone nonlinear dynamics.
import numpy as np
import scipy.io
import matplotlib.pyplot as plt
from sklearn.neural_network import MLPRegressor
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import r2_score, mean_squared_error
from scipy.signal import savgol_filter
import joblib



def load_and_preprocess(mat_path, skip_samples=0, dt=1/30.0, window_size=30):
    """
    Carga un .mat, aplica Savitzky-Golay, calcula aceleraciones y
    genera ventanas (X, Y) listas para entrenar.

    Vector de features por muestra:
        [ v_smooth[t]          (4)   <- velocidad actual
          u_hist[t-W:t].flat  (W*4)  <- historial de control (retardos)
          u_raw[t]            (4)  ] <- acción de control actual
    Total: 4 + W*4 + 4 = 4*(W+2) features
    (Para W=30 → 128 features, vs 244 del esquema anterior)

    skip_samples : cuántas muestras iniciales descartar (útil para
                   real_val.mat donde el dron estaba en el suelo).
    """
    print(f"\n📂 Cargando '{mat_path}' (skip={skip_samples} muestras)...")
    data = scipy.io.loadmat(mat_path)

    v_raw = np.vstack((
        data['vx_b'],
        data['vy_b'],
        data['vz_b'],
        data['vpsi']
    )).T
    u_raw = data['u']

    v_raw = v_raw[skip_samples:]
    u_raw = u_raw[skip_samples:]

    v_smooth = savgol_filter(v_raw, window_length=11, polyorder=3, axis=0)

    dv = np.diff(v_smooth, axis=0) / dt

    X_list, Y_list = [], []
    for t in range(window_size, len(dv)):
        v_now     = v_smooth[t].flatten()
        u_history = u_raw[t - window_size:t].flatten()
        u_actual  = u_raw[t].flatten()
        X_list.append(np.hstack((v_now, u_history, u_actual)))
        Y_list.append(dv[t])

    X = np.array(X_list)
    Y = np.array(Y_list)
    n_feat = X.shape[1]
    print(f"   → {len(X)} ventanas  |  X: {X.shape}  |  Y: {Y.shape}")
    print(f"   → Features: v_now(4) + u_hist({window_size}×4) + u_now(4) = {n_feat}")

    return X, Y, v_smooth, u_raw


DT          = 1 / 30.0
WINDOW_SIZE = 30

datasets = [
    ("train_values.mat", 0),
    ("real_val.mat",     10),   # primeras 10 muestras descartadas (dron en suelo)
    ("firstdata.mat",    0),
]

all_X, all_Y = [], []
val_data = {}

for mat_path, skip in datasets:
    X, Y, v_smooth, u = load_and_preprocess(
        mat_path, skip_samples=skip, dt=DT, window_size=WINDOW_SIZE
    )
    all_X.append(X)
    all_Y.append(Y)
    val_data[mat_path] = {"v_smooth": v_smooth, "u": u}

X_all = np.vstack(all_X)
Y_all = np.vstack(all_Y)
print(f"\n✅ Dataset combinado → X: {X_all.shape}  |  Y: {Y_all.shape}")



scaler_X = StandardScaler()
scaler_Y = StandardScaler()
X_scaled = scaler_X.fit_transform(X_all)
Y_scaled = scaler_Y.fit_transform(Y_all)



model = MLPRegressor(
    hidden_layer_sizes=(256, 128),
    activation='tanh',
    solver='adam',
    learning_rate_init=0.0005,
    alpha=0.005,
    batch_size=256,
    max_iter=50000,
    tol=1e-10,
    n_iter_no_change=200,
    early_stopping=True,
    validation_fraction=0.1,
    verbose=True
)

print("\n🚀 Entrenando red con los 3 datasets combinados...")
model.fit(X_scaled, Y_scaled)



model_filename = 'drone_model_realtime_v2.pkl'
checkpoint = {
    'model':       model,
    'scaler_X':    scaler_X,
    'scaler_Y':    scaler_Y,
    'window_size': WINDOW_SIZE,
    'dt':          DT
}
joblib.dump(checkpoint, model_filename)
print(f"\n✅ Modelo guardado como '{model_filename}'")



Y_pred_scaled = model.predict(X_scaled)
Y_pred        = scaler_Y.inverse_transform(Y_pred_scaled)

print("\n" + "=" * 50)
print(f"R² SCORE  (dataset combinado): {r2_score(Y_all, Y_pred):.4f}")
print(f"RMSE aceleración:              {np.sqrt(mean_squared_error(Y_all, Y_pred)):.6f}")
print("=" * 50)



def get_accel_mlp(v_now, u_hist_window, u_now):
    """
    v_now        : array (4,)   — velocidad en el instante t
    u_hist_window: array (W, 4) — historial de control [t-W:t]
    u_now        : array (4,)   — acción de control en t
    """
    feat        = np.hstack((v_now.flatten(),
                              u_hist_window.flatten(),
                              u_now.flatten()))
    feat_scaled = scaler_X.transform(feat.reshape(1, -1))
    accel_sc    = model.predict(feat_scaled)
    return scaler_Y.inverse_transform(accel_sc).flatten()


def validate_rk4(v_smooth, u, label):
    start_idx = WINDOW_SIZE
    v_sim     = np.zeros_like(v_smooth)
    v_sim[:start_idx] = v_smooth[:start_idx]

    print(f"\n🔄 Validación RK4 para '{label}'...")
    for t in range(start_idx - 1, len(v_smooth) - 1):
        u_h = u[t - WINDOW_SIZE + 1: t + 1]
        u_n = u[t + 1]

        def f(v_state):
            return get_accel_mlp(v_state, u_h, u_n)

        y  = v_sim[t]
        k1 = f(y)
        k2 = f(y + 0.5 * DT * k1)
        k3 = f(y + 0.5 * DT * k2)
        k4 = f(y + DT * k3)
        v_sim[t + 1] = y + (DT / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

    rmse = np.sqrt(mean_squared_error(v_smooth[start_idx:], v_sim[start_idx:]))
    print(f"   RMSE velocidad simulada: {rmse:.6f}")
    return v_sim


labels_plot = ["vx_b", "vy_b", "vz_b", "vpsi"]
fig, axes   = plt.subplots(
    4, len(datasets),
    figsize=(6 * len(datasets), 10),
    sharex='col'
)

for col, (mat_path, _) in enumerate(datasets):
    v_smooth = val_data[mat_path]["v_smooth"]
    u        = val_data[mat_path]["u"]
    v_sim    = validate_rk4(v_smooth, u, mat_path)

    for row in range(4):
        ax = axes[row, col]
        ax.plot(v_smooth[:, row], 'k',  alpha=0.4, linewidth=1,   label='Real')
        ax.plot(v_sim[:, row],    'b--', linewidth=0.8, label='Sim RK4')
        ax.set_ylabel(labels_plot[row])
        ax.grid(True, alpha=0.3)
        if row == 0:
            ax.set_title(mat_path)
        if row == 0 and col == 0:
            ax.legend(fontsize=8)

plt.suptitle("Validación RK4 — Red v2 (v_now + u_hist + u_now)", fontsize=13, y=1.01)
plt.tight_layout()
plt.savefig("validacion_rk4_v2.png", dpi=150, bbox_inches='tight')
plt.show()
print("\n📊 Gráfica guardada como 'validacion_rk4_v2.png'")