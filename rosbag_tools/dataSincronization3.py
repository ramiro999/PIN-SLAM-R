#!/usr/bin/env python3
import rosbag
import rospy
import numpy as np
import pyproj
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import NavSatFix, PointCloud2, PointField, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

class ROSBagProcessor:
    def __init__(self, input_bag_path, output_bag_path):
        """
        Inicializa el procesador de rosbag con rutas de entrada y salida
        
        Args:
            input_bag_path (str): Ruta del rosbag de entrada
            output_bag_path (str): Ruta del rosbag de salida
        """
        self.input_bag_path = input_bag_path
        self.output_bag_path = output_bag_path
        
        # Configuración de transformación de coordenadas para Colombia
        self.wgs84_transformer = pyproj.Transformer.from_crs(
            "EPSG:3116",  # Sistema de referencia local Colombia
            "EPSG:4326",  # WGS84 (Lat/Lon)
            always_xy=True
        )
        
        # Variables para almacenar el último mensaje GPS
        self.last_gps_msg = None
        self.last_gps_time = None
    
    def convert_xyz_to_wgs84(self, point_cloud_msg):
        """
        Convierte coordenadas XYZ a WGS84
        
        Args:
            point_cloud_msg (PointCloud2): Mensaje de nube de puntos original
        
        Returns:
            PointCloud2: Nube de puntos convertida a WGS84
        """
        # Extraer puntos del mensaje original
        pc_data = list(pc2.read_points(point_cloud_msg, 
                                       field_names=("x", "y", "z"), 
                                       skip_nans=True))
        
        # Convertir cada punto
        converted_points = []
        for point in pc_data:
            try:
                # Transformar coordenadas locales a lon, lat, alt
                lon, lat, alt = self.wgs84_transformer.transform(
                    point[0], point[1], point[2]
                )
                converted_points.append((lon, lat, alt))
            except Exception as e:
                rospy.logerr(f"Error convirtiendo punto: {e}")
                continue
        
        # Definir campos para el nuevo punto
        fields = [
            PointField('longitude', 0, PointField.FLOAT32, 1),
            PointField('latitude', 4, PointField.FLOAT32, 1),
            PointField('altitude', 8, PointField.FLOAT32, 1)
        ]
        
        # Crear nuevo header
        header = Header()
        header.frame_id = point_cloud_msg.header.frame_id
        header.stamp = point_cloud_msg.header.stamp
        
        # Crear nueva nube de puntos
        converted_cloud = pc2.create_cloud(
            header, 
            fields, 
            converted_points
        )
        
        return converted_cloud
    
    def find_closest_gps_msg(self, point_cloud_time, tolerance_secs=1.0):
        """
        Encuentra el mensaje GPS más cercano a una nube de puntos
        
        Args:
            point_cloud_time (rospy.Time): Tiempo de la nube de puntos
            tolerance_secs (float): Tolerancia máxima de tiempo en segundos
        
        Returns:
            NavSatFix: Mensaje GPS más cercano o None
        """
        if self.last_gps_msg is None:
            return None
        
        time_diff = abs((self.last_gps_time - point_cloud_time).to_sec())
        
        return self.last_gps_msg if time_diff <= tolerance_secs else None
    
    def process_rosbag(self):
        """
        Procesa el rosbag de entrada, sincroniza GPS y genera rosbag de salida
        """
        with rosbag.Bag(self.output_bag_path, 'w') as outbag:
            with rosbag.Bag(self.input_bag_path, 'r') as inbag:
                for topic, msg, t in inbag.read_messages():
                    # Guardar último mensaje GPS
                    if topic == '/gps/fix':
                        self.last_gps_msg = msg
                        self.last_gps_time = t
                        outbag.write(topic, msg, t)
                    
                    # Procesar nubes de puntos
                    elif topic == '/points_raw':
                        # Obtener mensaje GPS más cercano
                        closest_gps = self.find_closest_gps_msg(t)
                        
                        # Convertir nube de puntos
                        converted_point_cloud = self.convert_xyz_to_wgs84(msg)
                        outbag.write(topic, converted_point_cloud, t)
                    
                    # Copiar otros tópicos sin modificación
                    elif topic in ['/imu_correct', 'odometry/imu']:
                        outbag.write(topic, msg, t)
        
        rospy.loginfo(f"Rosbag procesado: {self.output_bag_path}")

def main():
    # Configuración de parámetros
    rospy.init_node('rosbag_gps_sync_processor', anonymous=True)
    
    # Obtener parámetros desde el lanzador o línea de comandos
    input_bag = rospy.get_param('~input_bag', '../config/lidar_hesai/park_dataset.bag')
    output_bag = rospy.get_param('~output_bag', './output_georef_2/rosbag_gps_synced.bag')
    
    try:
        processor = ROSBagProcessor(input_bag, output_bag)
        processor.process_rosbag()
    except Exception as e:
        rospy.logerr(f"Error procesando rosbag: {e}")
python3 analysisKML.py
/home/hover/miniconda3/lib/python3.12/site-packages/fastkml/config.py:39: UserWarning: Package `lxml` missing. Pretty print will be disabled
  warnings.warn("Package `lxml` missing. Pretty print will be disabled")  # noqa: B028
Traceback (most recent call last):
  File "/home/hover/PIN-SLAM-R/rosbag_tools/analysisKML.py", line 16, in <module>
    features = list(kml_doc.features())
                    ^^^^^^^^^^^^^^^^^^
TypeError: 'list' object is not callable
if __name__ == '__main__':
    main()