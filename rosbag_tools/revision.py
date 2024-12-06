import rosbag
from sensor_msgs.msg import NavSatFix, PointCloud2
import sensor_msgs.point_cloud2 as pc2

# Ruta del nuevo archivo rosbag
georef_bag_path = 'output_georef/georeferenced_dataset.bag'

print("Leyendo el rosbag georreferenciado...")
with rosbag.Bag(georef_bag_path, 'r') as bag:
    topics = bag.get_type_and_topic_info()[1].keys()
    print(f"Tópicos en el bag: {list(topics)}")

    for topic, msg, t in bag.read_messages(topics=['/points_raw']):
        if isinstance(msg, PointCloud2):
            points = list(pc2.read_points(msg, skip_nans=True))
            print(f"Primeros 5 puntos LiDAR del tópico '{topic}':")
            print(points[:5])
            break

    for topic, msg, t in bag.read_messages(topics=['/gps/fix']):
        if isinstance(msg, NavSatFix):
            print(f"Primer mensaje GPS del tópico '{topic}':")
            print(f"Latitud: {msg.latitude}, Longitud: {msg.longitude}, Altitud: {msg.altitude}")
            break
