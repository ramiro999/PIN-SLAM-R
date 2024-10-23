import rosbag

def downsample_bag(input_bag: str, output_bag: str, original_frequency: int, new_frequency: int):
    """
    Función para realizar downsampling de un archivo .bag

    :param input_bag: Ruta del archivo .bag de entrada
    :param output_bag: Ruta del archivo .bag de salida con la frecuencua reducida.
    :param original_frequency: Frecuencia original del archivo .bag (en Hz)
    :param new_frequency: Frecuencia deseada del archivo .bag de salida (en Hz)
    """

    # Factor de downsamping (cuántos mensajes debemos saltar)
    downsample_factor = original_frequency // new_frequency # -> floor division
    message_count = 0 

    # Abrir el archivo .bag original para leer
    with rosbag.Bag(input_bag, 'r') as inbag:
        # Crear el archivo .bag de salida
        with rosbag.Bag(output_bag, 'w') as outbag:
            for topic, msg, t in inbag.read_messages():
                if message_count % downsample_factor == 0:
                    outbag.write(topic, msg, t)
                message_count += 1

    print(f"Downsampling completado. Archivo guardado en: {output_bag}")

# Ejemplo de uso
input_bag = "config/lidar_hesai/comedorCompleto.bag"
output_bag = "config/lidar_hesai/comedorLowRate1.bag"
original_frequency = 1000 # Hz
new_frequency = 500 # Hz

downsample_bag(input_bag, output_bag, original_frequency, new_frequency)