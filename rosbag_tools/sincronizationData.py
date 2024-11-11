import rosbag 
import rospy
from sensor_msgs.msg import NavSatFix, PointCloud2
from rospy.rostime import Time 
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import pyproj
import laspy



# Lectura del rosbag
bag = rosbag.Bag('../config/lidar_hesai_park_dataset.bag')

# Creación de listas para almacenar los datos
gps_msgs = []
lidar_msgs = []

# Lectura de los mensajes del rosbag
for topic, msg, t in bag.read_messages(topics=['/gps/fix', '/points_raw']):
    if topic == '/gps/fix':
        gps_msgs.append((t.to_sec(), msg))
    elif topic == '/points_raw':
        lidar_msgs.append((t.to_sec(), msg))

bag.close()

# ----- Sincronización de los datos -----

# Ordenar los mensajes por tiempo
gps_msgs.sort(key=lambda x: x[0])
lidar_msgs.sort(key=lambda x: x[0])

# Inicializar la variables 
gps_index = 0
gps_len = len(gps_msgs)
synchorized_data = []

# Sincronizar los mensajes
for lidar_time, lidar_msg in lidar_msgs:
    # Avanzar en la lista de GPS hasta encontrar el último timespamp menor al del lidar
    while gps_index < gps_len - 1 and gps_msgs[gps_index + 1][0] <= lidar_time:
        gps_index += 1

    
    # Obtener el mensaje del GPS correspondiente
    gps_time, gps_msg = gps_msgs[gps_index]

    # Almacenar los datos sincronizados
    synchorized_data.append({
        'lidar_time': lidar_time,
        'lidar_msg': lidar_msg,
        'gps_time': gps_time,
        'gps_msg': gps_msg
    })


# ----- Procesamiento de los datos -----

"""
Con los datos sincronizados, podemos proceder a transformar las coordenadas de la nube de puntos
al sistema global utilizando los datos de GPS.
"""

# Configuración del sistema de coordenadas
wgs84 = pyproj.Proj(init='epsg:4326') # WGS84
ecef = pyproj.Proj(init='epsg:4978') # Earth-Centered, Earth-Fixed

# Procesar cada conjunto sincronizado de datos 
for data in synchorized_data:
    # Extraer los mensajes
    lidar_msg = data['lidar_msg']
    gps_msg = data['gps_msg']

    # Obtener la posición del sensor de coordenadas ECEF
    lon, lat, alt = gps_msg.longitude, gps_msg.latitude, gps_msg.altitude
    x0, y0, z0 = pyproj.transform(wgs84, ecef, lon, lat, alt)

    # Extraer los puntos de la nube de puntos
    points = pc2.read_points_list(lidar_msg, skip_nans=True)

    # Convertir los puntos a una matriz numpy
    points_array = np.array([[p[0], p[1], p[2]] for p in points])

    # Si es necesario, aplicar la rotación según la orientación del sensor

    # Trasladar los puntos al sistema global
    global_points = points_array + np.array([x0, y0, z0])


# ----- Almacenamiento de los datos -----

# Crear un archivo LAS
header = laspy.header.Header()
out_file = laspy.file.File('output.las', mode='w', header=header)

# Asignar las coordenadas
out_file.X = global_points[:, 0]
out_file.Y = global_points[:, 1]
out_file.Z = global_points[:, 2]

# Cerrar el archivo
out_file.close()

