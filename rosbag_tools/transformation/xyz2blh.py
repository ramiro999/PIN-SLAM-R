import argparse
import math
import open3d as o3d

A = 6378137.0
B = 6356752.314245

def xyz2blh(x, y, z):
    e = math.sqrt(1 - (B**2) / (A**2))
    longitude = math.atan2(y, x)
    xy_hypot = math.hypot(x, y)
    lat0 = 0
    latitude = math.atan(z / xy_hypot)
    while abs(latitude - lat0) > 1E-9:
        lat0 = latitude
        N = A / math.sqrt(1 - e**2 * math.sin(lat0)**2)
        latitude = math.atan((z + e**2 * N * math.sin(lat0)) / xy_hypot)
    N = A / math.sqrt(1 - e**2 * math.sin(latitude)**2)
    if abs(latitude) < math.pi / 4:
        R, phi = math.hypot(xy_hypot, z), math.atan(z / xy_hypot)
        height = R * math.cos(phi) / math.cos(latitude - N)
    else:
        height = z / math.sin(latitude) - N * (1 - e**2)
    longitude = math.degrees(longitude)
    latitude = math.degrees(latitude)
    return latitude, longitude, height

def read_pcd(file_path):
    return o3d.io.read_point_cloud(file_path)

def write_pcd(file_path, cloud):
    o3d.io.write_point_cloud(file_path, cloud)

def transform_and_save_pcd(input_pcd, output_pcd):
    cloud = read_pcd(input_pcd)
    points = cloud.points  
    transformed_points = []

    for point in points:
        x, y, z = point[0], point[1], point[2]
        lat, lon, hgt = xyz2blh(x, y, z)
        transformed_points.append([lat, lon, hgt])

    # Crear un nuevo PointCloud con los puntos transformados
    transformed_cloud = o3d.geometry.PointCloud()
    transformed_cloud.points = o3d.utility.Vector3dVector(transformed_points)
    write_pcd(output_pcd, transformed_cloud)

if __name__ == "__main__":
    input_pcd = "./filesPCD/globalmap_lidar_feature.pcd"
    output_pcd = "./filesPCD/output.pcd"
    transform_and_save_pcd(input_pcd, output_pcd)