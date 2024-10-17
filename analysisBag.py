import bagpy
from bagpy import bagreader
import pandas as pd 
import sensor_msgs.point_cloud2 as pc2
import rosbag

# leer el archivo .bag
bag = bagreader('config/lidar_hesai/comedorCompleto.bag')

# Verficar los topicos en el archivo .bag
print(bag.topic_table)

# Extraer datos del tópico de la nube de puntos
lidar = bag.message_by_topic('/hesai/pandar')
print(f"Datos extraídos en: {lidar}") 

# leer los datos de la nube de puntos
with rosbag.Bag('config/lidar_hesai/comedorCompleto.bag') as bag:
    for topic, msg, t in bag.read_messages(topics=['/hesai/pandar']):
        nube_puntos = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        print(f"Topic: {topic}")
        print(f"Message: {msg}")
        print(f"Time: {t}")
        break

# Crear un DataFrame con los datos de la nube de puntos
df = pd.DataFrame(nube_puntos, columns=['x', 'y', 'z'])

print(df.head())


# ---- Visualización de la nube de puntos ----
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.scatter(df['x'], df['y'], df['z'], c='r', marker='o', s=1)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Nube de puntos - hesai/pandar')

#plt.show()


# Extraer datos del tópico de datos IMU
imu_data = bag.message_by_topic('/imu/data')

df = pd.DataFrame(imu_data)

print(df.head())
print(df.describe())


print(f"Datos extraídos en: {imu_data}")

# leer los datos del tópico de datos IMU
with rosbag.Bag('config/lidar_hesai/comedorCompleto.bag') as bag:
    for topic, msg, t in bag.read_messages(topics=['/imu/data']):
        print(f"Topic: {topic}")
        print(f"Message: {msg}")
        print(f"Time: {t}")
        break

# Graficar los datos de aceleración (ax, ay, az)
import matplotlib.pyplot as plt

plt.figure()
plt.plot(df['Time'], df['linear_acceleration.x'], label='Acc X')
plt.plot(df['Time'], df['linear_acceleration.y'], label='Acc Y')
plt.plot(df['Time'], df['linear_acceleration.z'], label='Acc Z')
plt.xlabel('Tiempo (s)')
plt.ylabel('Aceleración (m/s²)')
plt.title('Aceleración IMU')
plt.legend()
plt.show()