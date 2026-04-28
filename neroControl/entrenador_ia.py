import rclpy
from stable_baselines3 import PPO
from bebop_env import BebopEnv

def main():
    # Inicialización única de rclpy
    if not rclpy.ok():
        rclpy.init()

    # Instanciar el entorno corregido
    env = BebopEnv()

    # Configuración PPO (MLP Policy 256x256)
    model = PPO(
        "MlpPolicy", 
        env, 
        verbose=1, 
        learning_rate=0.0003,
        n_steps=2048,
        batch_size=64,
        gamma=0.99,
        device="cpu"
    )

    print("--- INICIANDO ENTRENAMIENTO ESTABLE (30 HZ) ---")
    print("Sincronizado con Webots. Referencia fija por 10s.")
    
    try:
        model.learn(total_timesteps=300000, progress_bar=True)
        model.save("controlador_ia_final_v2")
        print("\n¡Éxito! Modelo guardado.")
    except KeyboardInterrupt:
        print("\nInterrupción. Guardando progreso...")
        model.save("controlador_ia_backup")
    finally:
        # Apagado seguro de ROS 2
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()