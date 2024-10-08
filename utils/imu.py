import numpy as np

try:
    from rosbags.typesys.types import sensor_msgs__msg__Imu as Imu
except ImportError as e:
    raise ImportError('rosbags library not installed, run "pip install -U rosbags"') from e

def read_imu_data(msg: Imu) -> dict:
    # Extract orientation, angular velocity and linear acceleration

    orientation = {
        "x": msg.orientation.x,
        "y": msg.orientation.y,
        "z": msg.orientation.z,
        "w": msg.orientation.w
    }

    orientation_covariance = np.array(msg.orientation_covariance).reshape(3, 3)

    angular_velocity = {
        "x": msg.angular_velocity.x,
        "y": msg.angular_velocity.y,
        "z": msg.angular_velocity.z
    }

    angular_velocity_covariance = np.array(msg.angular_velocity_covariance).reshape(3, 3)

    linear_acceleration = {
        "x": msg.linear_acceleration.x,
        "y": msg.linear_acceleration.y,
        "z": msg.linear_acceleration.z
    }

    linear_acceleration_covariance = np.array(msg.linear_acceleration_covariance).reshape(3, 3)

    timestamp = msg.header.stamp

    return {
        "orientation": orientation,
        "orientation_covariance": orientation_covariance,
        "angular_velocity": angular_velocity,
        "angular_velocity_covariance": angular_velocity_covariance,
        "linear_acceleration": linear_acceleration,
        "linear_acceleration_covariance": linear_acceleration_covariance,
        "timestamp": timestamp
    }

def print_imu_data(imu_data: dict):

    print(f"Timestamp: {imu_data['timestamp']}")

    print("Orientation:")
    print(f"x: {imu_data['orientation']['x']}")
    print(f"y: {imu_data['orientation']['y']}")
    print(f"z: {imu_data['orientation']['z']}")
    print(f"w: {imu_data['orientation']['w']}")

    print("Orientation Covariance:")
    print(imu_data['orientation_covariance'])

    print("Angular Velocity:")
    print(f"x: {imu_data['angular_velocity']['x']}")
    print(f"y: {imu_data['angular_velocity']['y']}")
    print(f"z: {imu_data['angular_velocity']['z']}")
    print(f"w: {imu_data['angular_velocity']['w']}")

    print("Angular Velocity Covariance:")
    print(imu_data['angular_velocity_covariance'])

    print("Linear Acceleration:")
    print(f"x: {imu_data['linear_acceleration']['x']}")
    print(f"y: {imu_data['linear_acceleration']['y']}")
    print(f"z: {imu_data['linear_acceleration']['z']}")

    print("Linear Acceleration Covariance:")
    print(imu_data['linear_acceleration_covariance'])


