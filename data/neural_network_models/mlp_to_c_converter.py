# Exports the trained Multi-Layer Perceptron (MLP) weights into a standalone C model.
import numpy as np
import joblib
import casadi as ca
import time
import subprocess

MODEL_PATH = "drone_model_realtime_v2.pkl"
checkpoint = joblib.load(MODEL_PATH)
mlp        = checkpoint['model']
scaler_X   = checkpoint['scaler_X']
scaler_Y   = checkpoint['scaler_Y']
WINDOW     = checkpoint['window_size']   # 30
DT         = checkpoint['dt']

weights = mlp.coefs_
biases  = [b.reshape(1, -1) for b in mlp.intercepts_]
mean_x, std_x = scaler_X.mean_.reshape(1, -1), scaler_X.scale_.reshape(1, -1)
mean_y, std_y = scaler_Y.mean_.reshape(1, -1), scaler_Y.scale_.reshape(1, -1)

n_in = weights[0].shape[0]
expected = 4 * (WINDOW + 2)
assert n_in == expected, f"Dimensión inesperada: {n_in} (se esperaban {expected})"
print(f"✅ Vector de entrada: v_now(4) + u_hist({WINDOW}×4) + u_now(4) = {n_in} features")

x = ca.MX.sym('x', 1, n_in)

h = (x - mean_x) / std_x
h = ca.tanh(ca.mtimes(h, weights[0]) + biases[0])
h = ca.tanh(ca.mtimes(h, weights[1]) + biases[1])
h = ca.mtimes(h, weights[2]) + biases[2]
y = h * std_y + mean_y

f_mlp = ca.Function('f_mlp', [x], [y])

c_file = "drone_model_v2.c"
f_mlp.generate(c_file)
print(f"✅ Archivo C generado: {c_file}")

so_file = "./drone_model_v2.so"
compile_command = f"gcc -fPIC -shared -O3 {c_file} -o {so_file}"
print("Compilando código C para máxima velocidad...")
subprocess.run(compile_command, shell=True, check=True)
print(f"✅ Modelo compilado: {so_file}")

f_compiled = ca.external('f_mlp', so_file)

test_input = np.random.randn(1, n_in)
n_tests = 10000

print(f"\nIniciando prueba de estrés ({n_tests} ejecuciones)...")
start_time = time.perf_counter()

for _ in range(n_tests):
    res = f_compiled(test_input)

end_time = time.perf_counter()

total_time   = end_time - start_time
avg_time_ms  = (total_time / n_tests) * 1000

print("="*45)
print(f"RESULTADOS MODELO COMPILADO (v2)")
print("="*45)
print(f"Features entrada: {n_in}  (antes: 244)")
print(f"Tiempo total:     {total_time:.4f} s")
print(f"Tiempo promedio:  {avg_time_ms:.6f} ms")
print(f"Frecuencia:       {1.0/(avg_time_ms/1000):.2f} Hz")
print("="*45)