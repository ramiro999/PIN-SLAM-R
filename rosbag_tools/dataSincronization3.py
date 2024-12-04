#!/usr/bin/python
import rosbag
import rospy
from sensor_msgs.msg import PointCloud2, NavSatFix, Imu
from nav_msgs.msg import Odometry
import numpy as np
from pyproj import Transformer
import sensor_msgs.point_cloud2 as pc2
import tf.transformations as tf

class RosbagGPSSync:
    def __init__(self):
        self.transformer = Transformer.from_crs(
            "epsg:4326",    # WGS84
            "epsg:32633",   # UTM zone 33N - ajusta según tu ubicación
            always_xy=True
        )
        
        # Almacenamiento de datos
        self.latest_gps = None
        self.latest_odom = None
        
    def interpolate_gps(self, points_time, gps_data):
        """
        Interpola datos GPS para un tiempo específico
        """
        if len(gps_data) < 2:
            return None
            
        # Encuentra los puntos GPS más cercanos en tiempo
        for i in range(len(gps_data) - 1):
            if gps_data[i][0] <= points_time <= gps_data[i + 1][0]:
                t0, gps0 = gps_data[i]
                t1, gps1 = gps_data[i + 1]
                
                # Interpolación lineal
                factor = (points_time - t0) / (t1 - t0)
                lat = gps0.latitude + factor * (gps1.latitude - gps0.latitude)
                lon = gps0.longitude + factor * (gps1.longitude - gps0.longitude)
                alt = gps0.altitude + factor * (gps1.altitude - gps0.altitude)
                
                return NavSatFix(
                    latitude=lat,
                    longitude=lon,
                    altitude=alt
                )
        return None

    def xyz_to_wgs84(self, x, y, z, ref_gps):
        """
        Convierte coordenadas XYZ locales a WGS84 usando un punto GPS de referencia
        """
        # Primero convertimos el GPS de referencia a UTM
        ref_e, ref_n = self.transformer.transform(ref_gps.longitude, ref_gps.latitude)
        
        # Sumamos el offset XY a las coordenadas UTM
        point_e = ref_e + x
        point_n = ref_n + y
        
        # Convertimos de vuelta a WGS84
        lon, lat = self.transformer.transform(point_e, point_n, direction='INVERSE')
        alt = ref_gps.altitude + z

        # Asegurar valores positivos
        lat = abs(lat)
        lon = abs(lon)
        alt = abs(alt)
        
        return lat, lon, alt

    def process_rosbag(self, input_bag_path, output_bag_path):
        # Almacenamiento temporal de datos GPS
        gps_buffer = []
        
        # Primer paso: recolectar datos GPS
        with rosbag.Bag(input_bag_path, 'r') as bag:
            for topic, msg, t in bag.read_messages(topics=['/gps/fix']):
                gps_buffer.append((t.to_sec(), msg))
        
        # Segundo paso: procesar todos los mensajes y crear nuevo rosbag
        with rosbag.Bag(input_bag_path, 'r') as inbag:
            with rosbag.Bag(output_bag_path, 'w') as outbag:
                for topic, msg, t in inbag.read_messages():
                    if topic == '/points_raw':
                        # Obtener GPS interpolado para este tiempo
                        interpolated_gps = self.interpolate_gps(t.to_sec(), gps_buffer)
                        
                        if interpolated_gps:
                            # Convertir la nube de puntos
                            points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
                            modified_points = []
                            
                            for x, y, z in points:
                                # Convertir cada punto a WGS84
                                lat, lon, alt = self.xyz_to_wgs84(x, y, z, interpolated_gps)
                                modified_points.append([lat, lon, alt])
                            
                            # Crear nuevo mensaje PointCloud2 con los puntos convertidos
                            # Aquí necesitarías implementar la creación del mensaje PointCloud2
                            # con los campos adicionales para lat, lon, alt
                            
                            # Escribir el mensaje modificado
                            outbag.write(topic, msg, t)
                            
                            # También guardamos el GPS interpolado
                            outbag.write('/gps/fix', interpolated_gps, t)
                    else:
                        # Copiar otros mensajes sin modificar
                        outbag.write(topic, msg, t)

def main():
    processor = RosbagGPSSync()
    
    # Ajusta estas rutas según tu sistema
    input_bag = "input.bag"
    output_bag = "output.bag"
    
    processor.process_rosbag(input_bag, output_bag)

if __name__ == '__main__':
    main()