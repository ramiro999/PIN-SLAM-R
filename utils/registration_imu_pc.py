"""
The objective this module is integrate point cloud and imu data 
"""

import sys 
from typing import Iterable, List, Optional, Tuple

import numpy as np
from point_cloud2 import read_point_cloud
from imu import read_imu_data

try: 
    from rosbags.typesys.types import sensor_msgs__msg__PointCloud2 as PointCloud2
    from rosbags.typesys.types import sensor_msgs__msg__PointField as PointField
    from rosbags.typesys.types import sensor_msgs__msg__Imu as Imu
except ImportError as e:
    raise ImportError('rosbags library not installed, run "pip install -U rosbags"') from e

def synchronize_imu_and_point_cloud(imu_msgs, lidar_msgs, max_time_diff=0.01):
    """Synchronize IMU and Point Cloud data by timestamp
    :param imu_msgs: List of IMU messages
    :param lidar_msgs: List of PointCloud2 messages
    :param max_time_diff: Maximun allowed time difference for synchronization
    :return: List of synchronized (imu_data, point_cloud_data) tuples
    """
    synchronize_data = []

    for lidar_msg in lidar_msgs: 
        lidar_time = lidar_msg.header.stamp.sec + lidar_msg.header.stamp.nanosec * 1e-9
        closest_imu_msg = min(
            imu_msgs,
            key=lambda imu_msg: abs((imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9) - lidar_time)
        )
        imu_time = closest_imu_msg.header.stamp.sec + closest_imu_msg.header.stamp.nanosec * 1e-9
        time_diff = abs(lidar_time - imu_time)

        if time_diff <= max_time_diff:
            imu_data = read_imu_data(closest_imu_msg)
            point_cloud, _ = read_point_cloud(lidar_msg)
            synchronize_data.append((imu_data, point_cloud))

    return synchronize_data

def apply_imu_to_point_cloud(imu_data, point_cloud):
    """Apply IMU transformation to point cloud points
    :param imu_data: IMU data dictionary
    :param point_cloud: Numpy array of point cloud (N x 3)
    :return: Transformed point cloud
    """

    # Extract quaternion from IMU data
    q = np.array([imu_data['orientation']['w'],
                  imu_data['orientation']['x'],
                  imu_data['orientation']['y'],
                  imu_data['orientation']['z']
                  ])
    
    # Convert quaternion to rotation matrix 
    R = quaternion_to_rotation_matrix(q)

    # Apply the rotation matrix to all points in the point cloud
    transformed_points = np.dot(point_cloud, R.T)

    return transformed_points

def quaternion_to_rotation_matrix(q):
    """ Convert quaternion to rotation matrix
    :param q: Quaternion (w, x, y, z)
    :return 3x3 rotation matrix
    """
    w,x,y,z = q

    return np.array([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
        [2*x*y + 2*w*z, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*w*x],
        [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x**2 - 2*y**2]
    ])

# Función principal (puede cambiar)
def process_bag_file(imu_msgs, lidar_msgs):
    synchronized_data = synchronize_imu_and_point_cloud(imu_msgs, lidar_msgs)

    for imu_data, point_cloud in synchronized_data:
        transformed_point_cloud = apply_imu_to_point_cloud(imu_data, point_cloud)

        # Aqui puedes guardar o procesar los datos transformados
        # Por ejemplo, escribir un archivo slam
        print(f"Processed {len(transformed_point_cloud)} points with corresponding IMU data.")
