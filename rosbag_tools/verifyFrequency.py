import rosbag

def verify_frequency(bag_file: str):
    """
    Función para verificar la frecuencia de los mensajes en un archivo .bag
    Calcula los intervalos de tiempo entre mensajes y estima la frecuencia.

    :param bag_file: Ruta del archivo .bag que se desea verificar
    """
    with rosbag.Bag(bag_file, 'r') as bag:
        timestamps = []
        for topic, msg, t in bag.read_messages():
            timestamps.append(t.to_sec()) # Obtener los tiempos en segundos

        # Calcular los intervalos de tiempo entre mensajes
        time_diffs = [t2 - t1 for t1, t2 in zip(timestamps[:-1], timestamps[1:])]
        average_time_diff = sum(time_diffs) / len(time_diffs)
        estimated_frequency = 1 / average_time_diff if average_time_diff > 0 else 0

        print(f"Frecuencia estimada: {estimated_frequency:.2f} Hz")
        print(f"Promedio de intervalo de tiempo entre mensajes: {average_time_diff:.6f} segundos")

# Ejemplo de uso
bag_file = "config/lidar_hesai/comedorLowRate1.bag"
verify_frequency(bag_file)