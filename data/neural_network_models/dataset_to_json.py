# Converts raw flight data datasets into structured JSON format for neural network training.
import joblib
import json

MODEL_PATH = "drone_model_realtime_v2.pkl"
checkpoint = joblib.load(MODEL_PATH)

WINDOW = int(checkpoint['window_size'])

metadata = {
    "model_name": "MLP Drone Realtime v2",
    "window_size": WINDOW,
    "dt": float(checkpoint['dt']),
    "input_format": f"v_now(4) + u_hist({WINDOW}x4) + u_now(4)",
    "input_dim": int(checkpoint['model'].coefs_[0].shape[0]),   # 4*(W+2) = 128
    "output_dim": int(checkpoint['model'].coefs_[-1].shape[1]), # 4
    "layers": [int(w.shape[1]) for w in checkpoint['model'].coefs_],
    "scaler_X": {
        "mean": checkpoint['scaler_X'].mean_.tolist(),
        "std":  checkpoint['scaler_X'].scale_.tolist()
    },
    "scaler_Y": {
        "mean": checkpoint['scaler_Y'].mean_.tolist(),
        "std":  checkpoint['scaler_Y'].scale_.tolist()
    }
}

JSON_PATH = "drone_metadata_v2.json"
with open(JSON_PATH, 'w') as f:
    json.dump(metadata, f, indent=4)

print(f"✅ Metadatos guardados en: {JSON_PATH}")
print(f"   input_dim  : {metadata['input_dim']}  (antes: 244)")
print(f"   output_dim : {metadata['output_dim']}")
print(f"   formato    : {metadata['input_format']}")