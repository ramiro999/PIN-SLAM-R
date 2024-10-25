import bagpy
from bagpy import bagreader
import pandas as pd
import sensor_msgs.point_cloud2 as pc2
import rosbag
import matplotlib.pyplot as plt

# Leer el archivo .bag
bag = bagreader('../config/lidar_hesai/campus_small_dataset.bag')

# Verificar los tópicos disponibles
print(bag.topic_table)

# Extraer datos del tópico de IMU
imu_data_path = bag.message_by_topic('/imu_correct')
print(f"Datos extraídos en: {imu_data_path}")

# Leer los datos del archivo CSV generado por bagpy
df = pd.read_csv(imu_data_path)

# Mostrar las primeras filas y estadísticas básicas
print(df.head())
print(df.describe())

# Quiero guardar esos datos en un archivo CSV

# Leer directamente desde el .bag (opcional, para mostrar ejemplos de mensajes)
with rosbag.Bag('../config/lidar_hesai/campus_small_dataset.bag', 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=['/imu_correct']):
        print(f"Topic: {topic}")
        print(f"Message: {msg}")
        print(f"Time: {t}")
        break  # Solo imprimimos el primer mensaje como ejemplo

# Graficar los datos de aceleración (ax, ay, az)
plt.figure()
plt.plot(df['Time'], df['linear_acceleration.x'], label='Acc X')
plt.plot(df['Time'], df['linear_acceleration.y'], label='Acc Y')
plt.plot(df['Time'], df['linear_acceleration.z'], label='Acc Z')

plt.xlabel('Tiempo (s)')
plt.ylabel('Aceleración (m/s²)')
plt.title('Aceleración IMU')
plt.legend()
plt.tight_layout()
plt.savefig('output_imus/aceleracion_imu.png')
plt.show()

# Graficar los datos de velocidad angular (gx, gy, gz)
plt.plot(df['Time'], df['angular_velocity.x'], label='Vel X')
plt.plot(df['Time'], df['angular_velocity.y'], label='Vel Y')
plt.plot(df['Time'], df['angular_velocity.z'], label='Vel Z')

plt.xlabel('Tiempo (s)')
plt.ylabel('Velocidad angular (rad/s)')
plt.title('Velocidad angular IMU')
plt.legend()
plt.tight_layout()
plt.savefig('output_imus/velocidad_angular_imu.png')
plt.show()

# Graficar los datos de orientación (qx, qy, qz, qw)
plt.plot(df['Time'], df['orientation.x'], label='Ori X')
plt.plot(df['Time'], df['orientation.y'], label='Ori Y')
plt.plot(df['Time'], df['orientation.z'], label='Ori Z')
plt.plot(df['Time'], df['orientation.w'], label='Ori W')

plt.xlabel('Tiempo (s)')
plt.ylabel('Orientación')
plt.title('Orientación IMU')
plt.legend()
plt.tight_layout()
plt.savefig('output_imus/orientacion_imu.png', dpi=300)
plt.show()

