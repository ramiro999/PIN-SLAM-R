import os
import rosbag
import bagpy 
from bagpy import bagreader
import sensor_msgs.point_cloud2 as pc2
import pandas as pd
import open3d as o3d

# Leer el archivo .bag
b = bagreader('comedorCompleto.bag')

# Mostrar los tópicos disponibles
print(f'Topics disponibles: {b.topics}')


# Extraer datos de nube de puntos
nube_topic = b.message_by_topic('/points_raw')
imu_topic = b.message_by_topic('/imu')
gnss_topic = b.message_by_topic('/gnss')

# Extraer mensajes en CSV
puntos_csv = b.message_by_topic('/points_raw', as_df=True)
imu_csv = b.message_by_topic('/imu', as_df=True)
gnss_csv = b.message_by_topic('/gnss', as_df=True)

# Verificar si los archivos se extrajeron correctamente
print(f'Nube de puntos guardada en: {puntos_csv}')
print(f'IMU guardada en: {imu_csv}')
print(f'GNSS guardada en: {gnss_csv}')

# Leer datos IMU y GNSS desde CSV usando pandas
imu_data = pd.read_csv(imu_csv)
gnss_data = pd.read_csv(gnss_csv)

print("Primeras filas de los datos IMU:")
print(imu_data.head())

print("Primeras filas de los datos GNSS:")
print(gnss_data.head())

# Procesar las nubes de puntos (convertir mensajes .csv a formato Open3D)

def csv_to_pcd(csv_file):
    df = pd.read_csv(csv_file)
    points = df[['x', 'y', 'z']].values # Asegúrate que las columnas sean correctas

    # Crear una nube de puntos en Open3D
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd

# Convertir la nube de puntos extraida en un archivo PCD
nube_pcd = csv_to_pcd(puntos_csv)
o3d.io.write_point_cloud("nube.pcd", nube_pcd)

print("Nube de puntos convertida a formato PCD y guardada como 'nube.pcd'")

# Cargar y visualizar la nube de puntos con Open3D
nube_cargada = o3d.io.read_point_cloud("nube.pcd")
o3d.visualization.draw_geometries([nube_cargada])

print("Visualización completa.")