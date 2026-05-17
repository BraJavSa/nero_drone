import numpy as np
import scipy.io
import matplotlib.pyplot as plt
import joblib
from sklearn.metrics import r2_score, mean_squared_error
from scipy.signal import savgol_filter

# 1. CONFIGURACIÓN DE RUTAS
MODEL_PATH = "drone_model_realtime_v2.pkl"
MAT_PATH = "firstdata.mat"

def validate():
    # 2. CARGAR MODELO Y ESCALADORES
    print(f"Cargando modelo desde: {MODEL_PATH}...")
    try:
        checkpoint = joblib.load(MODEL_PATH)
        model = checkpoint['model']
        scaler_X = checkpoint['scaler_X']
        scaler_Y = checkpoint['scaler_Y']
        window_size = checkpoint['window_size']
        dt = checkpoint['dt']
    except FileNotFoundError:
        print("❌ Error: No se encontró el archivo .pkl. Asegúrate de haber entrenado el modelo primero.")
        return

    # 3. CARGAR DATOS PARA PRUEBA
    print(f"Cargando datos de prueba desde: {MAT_PATH}...")
    data = scipy.io.loadmat(MAT_PATH)

    v_real = np.vstack((data['vx_b'], data['vy_b'], data['vz_b'], data['vpsi'])).T
    u = data['u']

    # Suavizado idéntico al entrenamiento para una comparación justa
    v_smooth = savgol_filter(v_real, window_length=11, polyorder=3, axis=0)

    # 4. PREPARACIÓN DE LA SIMULACIÓN FREE-RUN
    num_samples = len(v_smooth)
    v_sim = np.zeros_like(v_smooth)
    v_sim[:window_size] = v_smooth[:window_size]

    def get_accel(v_now, u_hist, u_now):
        """
        v_now  : array (4,)    — velocidad en el instante t (simulada)
        u_hist : array (W, 4)  — historial de control [t-W:t]
        u_now  : array (4,)    — acción de control en t
        """
        features = np.hstack((v_now.flatten(), u_hist.flatten(), u_now.flatten()))
        X_scaled = scaler_X.transform(features.reshape(1, -1))
        Y_scaled = model.predict(X_scaled)
        return scaler_Y.inverse_transform(Y_scaled).flatten()

    # 5. BUCLE DE SIMULACIÓN RK4
    print("Iniciando simulación Free-Run (esto puede tardar unos segundos)...")
    for t in range(window_size - 1, num_samples - 1):
        u_window = u[t - window_size + 1 : t + 1]   # (W, 4)
        u_actual = u[t + 1]                          # (4,)

        def f(v_state):
            # Ahora solo se pasa v_state directamente como velocidad actual
            return get_accel(v_state, u_window, u_actual)

        y_n = v_sim[t]
        k1 = f(y_n)
        k2 = f(y_n + 0.5 * dt * k1)
        k3 = f(y_n + 0.5 * dt * k2)
        k4 = f(y_n + dt * k3)

        v_sim[t + 1] = y_n + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

    # 6. CÁLCULO DE MÉTRICAS FINALES
    v_real_eval = v_smooth[window_size:]
    v_sim_eval  = v_sim[window_size:]

    rmse = np.sqrt(mean_squared_error(v_real_eval, v_sim_eval))
    mae  = np.mean(np.abs(v_real_eval - v_sim_eval))

    print("\n" + "="*45)
    print("       RESULTADOS DE LA VALIDACIÓN")
    print("="*45)
    print(f"RMSE Total: {rmse:.6f}")
    print(f"MAE Total:  {mae:.6f}")
    print("="*45)

    # 7. GRÁFICAS DE COMPARACIÓN
    fig, axs = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    labels = ['VX (m/s)', 'VY (m/s)', 'VZ (m/s)', 'V-PSI (rad/s)']
    colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red']

    for i in range(4):
        axs[i].plot(v_smooth[:, i], color='black', alpha=0.4, label='Real (Data)')
        axs[i].plot(v_sim[:, i], color=colors[i], linestyle='--', label=f'Simulado {labels[i]}')
        axs[i].axvline(x=window_size, color='gray', linestyle=':', alpha=0.5)
        axs[i].set_ylabel(labels[i])
        axs[i].legend(loc='upper right')
        axs[i].grid(True, alpha=0.3)

    plt.xlabel('Muestras (Samples)')
    plt.suptitle(f'Validación de Modelo: {MODEL_PATH}', fontsize=14)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    validate()