import bagpy
from bagpy import bagreader
import pandas as pd
import sensor_msgs.point_cloud2 as pc2
import rosbag
import matplotlib.pyplot as plt
import os

# Leer el archivo .bag
bag = bagreader('../config/lidar_hesai/park_dataset.bag')

# Verificar los tópicos disponibles
print(bag.topic_table)

# Extraer datos del tópico de GPS
gps_data = bag.message_by_topic('/gps/fix')
print(f"Datos extraídos en: {gps_data}")

# Leer los datos del archivo CSV generado por bagpy
df = pd.read_csv(gps_data)

# Mostrar las primeras filas y estadisticas básicas
print(df.head())

# Mostrar solo dos columnas para latitude y longitude
print(df[['latitude', 'longitude']])

# Crear la carpeta output si no existe
if not os.path.exists('output_gps'):
    os.makedirs('output_gps')

# Guardar el csv con todas las columnas
df.to_csv('output_gps/gps_data.csv', index=False)

# Graficar los datos de latitude, longitude y altitude
plt.figure()
plt.plot(df['Time'], df['latitude'], label='Latitude')
plt.plot(df['Time'], df['longitude'], label='Longitude')
plt.plot(df['Time'], df['altitude'], label='Altitude')

plt.xlabel('Tiempo (s)')
plt.ylabel('Coordenadas')
plt.title('Datos de GPS')
plt.legend()
plt.tight_layout()
plt.savefig('output_gps/gps_data.png') 
plt.show()