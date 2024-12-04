import rosbag
import rospy
from sensor_msgs.msg import NavSatFix, PointCloud2, Imu
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from pyproj import CRS, Transformer
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
imu_topic = '/imu/data'

# Sistemas de coordenadas
wgs84 = CRS.from_epsg(4326)  # Coordenadas geográficas WGS84
ecef = CRS.from_epsg(4978)  # Coordenadas ECEF (Earth-Centered, Earth-Fixed)
transformer = Transformer.from_crs(wgs84, ecef, always_xy=True)

# Ruta de salida para el nuevo archivo ROS Bag
output_dir = 'output_georef'
os.makedirs(output_dir, exist_ok=True)
output_bag_path = os.path.join(output_dir, 'georeferenced_dataset_with_imu.bag')

# ----- Lectura del rosbag -----

print("Leyendo rosbag...")
gps_msgs = []
lidar_msgs = []
imu_msgs = []

with rosbag.Bag(bag_path, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=[gps_topic, lidar_topic, imu_topic]):
        if topic == gps_topic:
            gps_msgs.append((t.to_sec(), msg))
        elif topic == lidar_topic:
            lidar_msgs.append((t.to_sec(), msg))
        elif topic == imu_topic:
            imu_msgs.append((t.to_sec(), msg))

print("Mensajes cargados.")

# Ordenar los mensajes por tiempo
gps_msgs.sort(key=lambda x: x[0])
lidar_msgs.sort(key=lambda x: x[0])

# ----- Sincronización de los datos GPS y LiDAR -----

print("Sincronizando datos GPS y LiDAR...")
synchronized_data = []
gps_index = 0
gps_len = len(gps_msgs)

for lidar_time, lidar_msg in lidar_msgs:
    while gps_index < gps_len - 1 and gps_msgs[gps_index + 1][0] <= lidar_time:
        gps_index += 1

    gps_time, gps_msg = gps_msgs[gps_index]
    synchronized_data.append({
        'lidar_time': lidar_time,
        'lidar_msg': lidar_msg,
        'gps_time': gps_time,
        'gps_msg': gps_msg
    })

print(f"Datos sincronizados: {len(synchronized_data)} conjuntos.")

# ----- Creación del nuevo ROS Bag -----

print("Creando nuevo rosbag georreferenciado...")
with rosbag.Bag(output_bag_path, 'w') as out_bag:
    for data in synchronized_data:
        lidar_msg = data['lidar_msg']
        gps_msg = data['gps_msg']

        # Convertir coordenadas GPS a ECEF
        lon, lat, alt = gps_msg.longitude, gps_msg.latitude, gps_msg.altitude
        x0, y0, z0 = transformer.transform(lon, lat, alt)

        # Leer los puntos de la nube de puntos LiDAR
        points = list(pc2.read_points(lidar_msg, skip_nans=False))  # Leer todos los puntos (incluso con NaN)

        # Crear una lista para los puntos globales georreferenciados
        global_points = []
        for point in points:
            # Verificar que el punto no tenga valores NaN o infinitos
            if not any(np.isnan(value) or np.isinf(value) for value in point[:3]):
                # Transformar las coordenadas (x, y, z)
                x, y, z = point[:3]
                transformed_point = (x + x0, y + y0, z + z0) + point[3:]  # Conservar otros atributos
                global_points.append(transformed_point)

        # Crear un nuevo mensaje PointCloud2 con los puntos válidos
        fields = lidar_msg.fields  # Mantener los mismos campos que el original
        header = lidar_msg.header  # Mantener el mismo encabezado
        global_lidar_msg = pc2.create_cloud(header, fields, global_points)

        # Escribir los mensajes GPS y LiDAR transformados al nuevo bag
        out_bag.write(gps_topic, gps_msg, rospy.Time.from_sec(data['gps_time']))
        out_bag.write(lidar_topic, global_lidar_msg, rospy.Time.from_sec(data['lidar_time']))

    # Agregar los datos IMU sin modificar
    for imu_time, imu_msg in imu_msgs:
        out_bag.write(imu_topic, imu_msg, rospy.Time.from_sec(imu_time))

print(f"Archivo rosbag georreferenciado con datos IMU guardado en: {output_bag_path}")
