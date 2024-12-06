import argparse
import math 

A = 6378137.0
B = 6356752.314245

def xyz2blh(x, y, z):
    e = math.sqrt(1 - (B**2)/(A**2))
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
        R, phi = math.hypot(xy_hypot, z), math_atan(z / xy_hypot)
        height = R * math.cos(phi) / math.cos(latitude - N)
    else:
        height = z / math.sin(latitude) - N * (1 - e**2)
    longitude = math.degrees(longitude)
    latitude = math.degrees(latitude)
    return latitude, longitude, height

def read_pcd(file_path):
    return pcl.load(file_path)

def write_pcd(file_path, cloud):
    pcl.save(cloud, file_path)

def transform_and_save_pcd(input_pcd, output_pcd):
    cloud = read_pcd(input_pcd)
    transformed_cloud = []

    for point in cloud:
        x, y, z = point[0], point[1], point[2]
        lat, lon, hgt = xyz2blh(x, y, z)
        transformed_cloud.append([lat, lon, hgt])

    transformed_cloud = pcl.PointCloud(transformed_cloud)
    write_pcd(output_pcd, transformed_cloud)

if __name__ == "__main__":
    input_pcd = "input.pcd"
    output_pcd = "output.pcd"
    transform_and_save_pcd(input_pcd, output_pcd)