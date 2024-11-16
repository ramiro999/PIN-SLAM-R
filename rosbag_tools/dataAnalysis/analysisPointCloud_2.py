import rosbag
import sensor_msgs.point_cloud2 as pc2
import csv
import os
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.ticker as ticker

# Limitar el tamaño del archivo .csv a menos de 100 MB
def convert_bag_to_limited_csv(bag_path, csv_path, max_size_mb):
    with rosbag.Bag(bag_path, 'r') as bag, open(csv_path, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(['x', 'y', 'z', 'intensity', 'ring', 'timestamp'])
        total_size = 0
        for topic, msg, t in bag.read_messages(topics=['/hesai/pandar']):
            for point in pc2.read_points(msg, field_names=("x", "y", "z","intensity", 'ring', 'timestamp'), skip_nans=True):
                csv_writer.writerow(point)
                total_size = os.path.getsize(csv_path) / (1024 * 1024)  # Convertir a MB
                if total_size >= max_size_mb:
                    print(f"Archivo .csv alcanzó el tamaño límite de {max_size_mb} MB")
                    return

# Ruta del archivo .bag y .csv
bag_path = 'output_bags/output_splt_part_1.bag'
csv_path = 'outputs_csv/output_splt_part_1.csv'
max_size_mb = 90

"""
Lectura del archivo .csv y visualización de la nube de puntos
"""

# Convertir el archivo .bag a un archivo .csv limitado
convert_bag_to_limited_csv(bag_path, csv_path, max_size_mb)

# Leer el archivo .csv generado
df = pd.read_csv(csv_path)

print(df.head(100))
print(df.describe())

# ---- Visualización de la nube de puntos ----
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Ajustar los limites de los ejes
x_min, x_max = df['x'].min(), df['x'].max()
y_min, y_max = df['y'].min(), df['y'].max()
z_min, z_max = df['z'].min(), df['z'].max()

ax.set_xlim([x_min, x_max])
ax.set_ylim([y_min, y_max])
ax.set_zlim([z_min, z_max])

# Graficar la nube de puntos
ax.scatter(df['x'], df['y'], df['z'], c='r', marker='o', s=1)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Nube de puntos - hesai/pandar')
plt.show()

# --- KPIs respecto a las variables del csv ---

# 1. Calcular la cantidad de puntos en la nube de puntos
num_points = len(df)
print(f"La nube de puntos tiene {num_points} puntos")

# 2. Calcular la media de la intensidad de los puntos
mean_intensity = df['intensity'].mean()
print(f"La media de la intensidad de los puntos es: {mean_intensity}")

# 3. Calcular la media de la altura de los puntos
mean_height = df['z'].mean()
print(f"La media de la altura de los puntos es: {mean_height}")

# 4. Calcular la media de la distancia de los puntos al origen
df['distance'] = (df['x']**2 + df['y']**2 + df['z']**2)**0.5
mean_distance = df['distance'].mean()
print(f"La media de la distancia de los puntos al origen es: {mean_distance}")

# 5. Calcular la cantidad de anillos (rings) en la nube de puntos
num_rings = df['ring'].nunique()
print(f"La nube de puntos tiene {num_rings} anillos")

# 6. Calcular la cantidad de puntos por anillo
points_per_ring = df.groupby('ring').size()
print(f"Cantidad de puntos por anillo:\n{points_per_ring}")

# 7. Calcular la cantidad de puntos por intensidad
points_per_intensity = df.groupby('intensity').size()
print(f"Cantidad de puntos por intensidad:\n{points_per_intensity}")

# 8. Calcular la cantidad de puntos por altura
points_per_height = df.groupby('z').size()
print(f"Cantidad de puntos por altura:\n{points_per_height}")

# 9. Calcular la cantidad de puntos por distancia al origen
points_per_distance = df.groupby('distance').size()
print(f"Cantidad de puntos por distancia al origen:\n{points_per_distance}")

# 10. Calcular la cantidad de puntos por coordenada X
points_per_x = df.groupby('x').size()
print(f"Cantidad de puntos por coordenada X:\n{points_per_x}")

# 11. Calcular la cantidad de puntos por coordenada Y
points_per_y = df.groupby('y').size()
print(f"Cantidad de puntos por coordenada Y:\n{points_per_y}")

# 12. Calcular la cantidad de puntos por coordenada Z
points_per_z = df.groupby('z').size()
print(f"Cantidad de puntos por coordenada Z:\n{points_per_z}")

# 13. Calcular la cantidad de puntos por timestamp
points_per_timestamp = df.groupby('timestamp').size()
print(f"Cantidad de puntos por timestamp:\n{points_per_timestamp}")

# 14. Calcular la cantidad de puntos por anillo y por intensidad
points_per_ring_intensity = df.groupby(['ring', 'intensity']).size()
print(f"Cantidad de puntos por anillo e intensidad:\n{points_per_ring_intensity}")

# 15. Calcular la cantidad de puntos por anillo y por altura
points_per_ring_height = df.groupby(['ring', 'z']).size()
print(f"Cantidad de puntos por anillo y altura:\n{points_per_ring_height}")

# 16. Calcular la cantidad de puntos por anillo y por distancia al origen
points_per_ring_distance = df.groupby(['ring', 'distance']).size()
print(f"Cantidad de puntos por anillo y distancia al origen:\n{points_per_ring_distance}")

# 17. Calcular la cantidad de puntos por intensidad y por altura
points_per_intensity_height = df.groupby(['intensity', 'z']).size()
print(f"Cantidad de puntos por intensidad y altura:\n{points_per_intensity_height}")

# 18. Calcular la cantidad de puntos por intensidad y por distancia al origen
points_per_intensity_distance = df.groupby(['intensity', 'distance']).size()
print(f"Cantidad de puntos por intensidad y distancia al origen:\n{points_per_intensity_distance}")

# 19. Calcular la cantidad de puntos por altura y por distancia al origen
points_per_height_distance = df.groupby(['z', 'distance']).size()
print(f"Cantidad de puntos por altura y distancia al origen:\n{points_per_height_distance}")

# 20. Calcular la cantidad de puntos por anillo, intensidad y altura
points_per_ring_intensity_height = df.groupby(['ring', 'intensity', 'z']).size()
print(f"Cantidad de puntos por anillo, intensidad y altura:\n{points_per_ring_intensity_height}")

# 21. Calcular la cantidad de puntos por anillo, intensidad y distancia al origen
points_per_ring_intensity_distance = df.groupby(['ring', 'intensity', 'distance']).size()
print(f"Cantidad de puntos por anillo, intensidad y distancia al origen:\n{points_per_ring_intensity_distance}")

# 22. Calcular la cantidad de puntos por intensidad, altura y distancia al origen
points_per_intensity_height_distance = df.groupby(['intensity', 'z', 'distance']).size()
print(f"Cantidad de puntos por intensidad, altura y distancia al origen:\n{points_per_intensity_height_distance}")

# 23. Calcular la cantidad de puntos por anillo, intensidad, altura y distancia al origen
points_per_ring_intensity_height_distance = df.groupby(['ring', 'intensity', 'z', 'distance']).size()
print(f"Cantidad de puntos por anillo, intensidad, altura y distancia al origen:\n{points_per_ring_intensity_height_distance}")

# 24. Calcular la cantidad de puntos por coordenada X, Y y Z
points_per_xyz = df.groupby(['x', 'y', 'z']).size()
print(f"Cantidad de puntos por coordenada X, Y y Z:\n{points_per_xyz}")

# -- Visualización de los KPIs --