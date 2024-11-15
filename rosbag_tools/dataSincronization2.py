import rosbag
import rospy
from sensor_msgs.msg import NavSatFix, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from pyproj import CRS, Transformer
import laspy
import os

# Ignorar warnings innecesarios
import warnings
warnings.filterwarnings("ignore", category=FutureWarning)

# ----- Configuración -----

# Ruta del archivo rosbag
bag_path = '../config/lidar_hesai/park_dataset.bag'

# Tópicos del rosbag
gps_topic = '/gps/fix'
lidar_topic = '/points_raw'

# Sistemas de coordenadas
wgs84 = CRS.from_epsg(4326)  # Coordenadas geográficas WGS84
ecef = CRS.from_epsg(4978)  # Coordenadas ECEF (Earth-Centered, Earth-Fixed)
transformer = Transformer.from_crs(wgs84, ecef, always_xy=True)

# Ruta de salida para el archivo LAS
output_dir = 'output_gps'
os.makedirs(output_dir, exist_ok=True)
output_path = os.path.join(output_dir, 'output.las')

# ----- Lectura del rosbag -----

print("Leyendo rosbag...")
gps_msgs = []
lidar_msgs = []

with rosbag.Bag(bag_path, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=[gps_topic, lidar_topic]):
        if topic == gps_topic:
            gps_msgs.append((t.to_sec(), msg))
        elif topic == lidar_topic:
            lidar_msgs.append((t.to_sec(), msg))

print("Mensajes cargados.")

# Ordenar los mensajes por tiempo
gps_msgs.sort(key=lambda x: x[0])
lidar_msgs.sort(key=lambda x: x[0])

# ----- Sincronización de los datos -----

print("Sincronizando datos...")
synchorized_data = []
gps_index = 0
gps_len = len(gps_msgs)

for lidar_time, lidar_msg in lidar_msgs:
    # Encontrar el GPS más cercano en tiempo
    while gps_index < gps_len - 1 and gps_msgs[gps_index + 1][0] <= lidar_time:
        gps_index += 1

    gps_time, gps_msg = gps_msgs[gps_index]
    synchorized_data.append({
        'lidar_time': lidar_time,
        'lidar_msg': lidar_msg,
        'gps_time': gps_time,
        'gps_msg': gps_msg
    })

print(f"Datos sincronizados: {len(synchorized_data)} conjuntos.")

# ----- Procesamiento de los datos -----

print("Procesando datos...")
all_global_points = []

for data in synchorized_data:
    lidar_msg = data['lidar_msg']
    gps_msg = data['gps_msg']

    # Convertir coordenadas GPS a ECEF
    lon, lat, alt = gps_msg.longitude, gps_msg.latitude, gps_msg.altitude
    x0, y0, z0 = transformer.transform(lon, lat, alt)

    # Leer los puntos de la nube de puntos LiDAR
    points = pc2.read_points_list(lidar_msg, field_names=('x', 'y', 'z'), skip_nans=True)
    points_array = np.array([[p[0], p[1], p[2]] for p in points])

    # Trasladar los puntos al sistema global (ECEF)
    global_points = points_array + np.array([x0, y0, z0])
    all_global_points.append(global_points)

# Concatenar todos los puntos procesados
all_global_points = np.vstack(all_global_points)

print(f"Puntos procesados: {all_global_points.shape[0]}")

# ----- Almacenamiento en formato LAS -----

print("Guardando archivo LAS...")
header = laspy.LasHeader(point_format=3, version="1.2")
las = laspy.LasData(header)

# Asignar las coordenadas globales al archivo LAS
las.x = all_global_points[:, 0]
las.y = all_global_points[:, 1]
las.z = all_global_points[:, 2]

# Guardar el archivo
las.write(output_path)

print(f"Archivo LAS guardado en: {output_path}")
