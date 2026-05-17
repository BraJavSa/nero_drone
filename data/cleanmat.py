import scipy.io as sio
import os
import numpy as np

def trim_mat_file(input_name='real_train.mat', output_name='train_values.mat', seconds_to_remove=10):
    if not os.path.exists(input_name):
        print(f"Error: No se encuentra el archivo {input_name}")
        return

    # 1. Cargar el archivo original
    data = sio.loadmat(input_name)
    
    # 2. Obtener la frecuencia (hz) guardada en el archivo
    # En tu script original guardaste "hz": 30.0
    hz = float(data['hz'][0][0])
    
    # Calcular cuántas muestras (filas) equivalen a los segundos deseados
    samples_to_remove = int(seconds_to_remove * hz)
    total_samples = len(data['K'][0])
    
    if samples_to_remove >= total_samples:
        print("Error: El tiempo a eliminar es mayor o igual a la duración del log.")
        return

    new_end = total_samples - samples_to_remove
    print(f"Recortando {seconds_to_remove}s ({samples_to_remove} muestras).")
    print(f"Muestras originales: {total_samples} -> Muestras finales: {new_end}")

    # 3. Crear el nuevo diccionario con los datos recortados
    # Iteramos sobre las llaves para recortar todos los arrays (u, x_i, vx_b, etc.)
    trimmed_data = {}
    for key, value in data.items():
        # Solo procesamos los datos que no son metadatos de MATLAB (__version__, etc.)
        if not key.startswith('__'):
            # Si es un array de numpy, lo recortamos
            if isinstance(value, np.ndarray):
                # Verificamos si el array es horizontal o vertical para recortar correctamente
                if value.shape[0] == total_samples:
                    trimmed_data[key] = value[:new_end, :]
                elif value.shape[1] == total_samples:
                    trimmed_data[key] = value[:, :new_end]
                else:
                    trimmed_data[key] = value
            else:
                trimmed_data[key] = value

    # 4. Guardar el nuevo archivo
    sio.savemat(output_name, trimmed_data)
    print(f"Éxito: Archivo guardado como {output_name}")

if __name__ == "__main__":
    trim_mat_file()