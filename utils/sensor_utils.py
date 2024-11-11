import numpy as np
from sensor_msgs.msg import Imu, PointCloud2
from scipy.spatial.transform import Rotation as R

class IMUPreintegrator:
    """
    Clase para realizar la preintegración de los datos IMU.
    Se usa para calcular la orientación, velocidad y posición utilizando mediciones acelerométricas y giroscópicas.
    """
    def __init__(self, config):
        # Inicializar parámetros de la preintegración, como matrices.
        self.delta_position = np.zeros(3)
        self.delta_velocity = np.zeros(3)
        self.delta_orientation = R.from_quat([0, 0, 0, 1])
        self.config = config

        # Inicializar el bias de acelerómetro y giroscopio.
        # Reemplazar config.get() por acceder directamente a los atributos de la clase Config
        self.acc_bias = getattr(config, 'acc_bias', np.zeros(3))
        self.gyro_bias = getattr(config, 'gyro_bias', np.zeros(3))

        # Inicializar la covarianza de la preintegración
        self.covariance = np.eye(9)  # Covarianza (inicial)

    def integrate(self, imu_data):
        """
        Realiza la preintegración de los datos IMU.

        Parameters:
        imu_data: Imu
            Mensaje del tipo sensor_msgs/Imu con datos como aceleración, velocidad angular y orientación.
        """
        if not isinstance(imu_data, Imu):
            raise TypeError("Se esperaba un mensaje de tipo sensor_msgs/Imu.")

        # Obtener los datos del IMU
        dt = imu_data.header.stamp.to_sec()  # Tiempo
        acc = np.array([imu_data.linear_acceleration.x,
                        imu_data.linear_acceleration.y,
                        imu_data.linear_acceleration.z]) - self.acc_bias  # Aceleración lineal
        gyro = np.array([imu_data.angular_velocity.x,
                         imu_data.angular_velocity.y,
                         imu_data.angular_velocity.z]) - self.gyro_bias  # Velocidad angular

        # Actualizar la orientación usando la velocidad angular
        delta_rotation = R.from_rotvec(gyro * dt)  # Convertir velocidad
        self.delta_orientation = self.delta_orientation * delta_rotation

        # Obtener la matriz de rotación para proyectar la aceleración al marco global
        rotation_matrix = self.delta_orientation.as_matrix()

        # Actualizar la velocidad y posición usando la aceleración proyectada al marco global.
        global_acc = rotation_matrix @ acc  # Proyectar aceleración al marco global
        self.delta_velocity += global_acc * dt
        self.delta_position += self.delta_velocity * dt + 0.5 * global_acc * (dt ** 2)

        # Actualizar la covarianza (simplificada)
        F = np.eye(9)  # Matriz de transición de estado
        F[0:3, 3:6] = np.eye(3) * dt
        F[3:6, 6:9] = -rotation_matrix * dt
        self.covariance = F @ self.covariance @ F.T + self.config.process_noise * dt

        return {
            'position': self.delta_position,
            'velocity': self.delta_velocity,
            'orientation': self.delta_orientation.as_quat(),
            'covariance': self.covariance
        }

    def reset(self):
        """
        Reinicia los valores acumulados de la preintegración.
        """
        self.delta_position = np.zeros(3)
        self.delta_velocity = np.zeros(3)
        self.delta_orientation = R.from_quat([0, 0, 0, 1])
        self.covariance = np.eye(9)


class EKFProcessor:
    """
    Clase para realizar la fusión de datos GNSS e IMU utilizando un filtro de Kalman Extendido.
    """
    def __init__(self, config):
        # Inicializar el filtro EKF
        self.state = np.zeros(9)  # Ej.posición (x, y, z), velocidad (vx, vy, vz), orientación (roll, pitch, yaw)
        self.covariance = np.eye(9)  # Covarianza
        self.config = config

    def update(self, imu_data=None, gnss_data=None):
        """
        Actualiza el estado y la covarianza del sistema utilizando datos IMU y GNSS
        
        Parameters:
        imu_data: Imu
            Mensaje del tipo sensor_msgs/Imu con datos de aceleración y velocidad angular.
        gnss_data: dict
            Diccionario con datos de posición global (3D).
        """
        if imu_data:
            if not isinstance(imu_data, Imu):
                raise TypeError("Se esperaba un mensaje de tipo sensor_msgs/Imu.")
            dt = imu_data.header.stamp.to_sec()
            acc = np.array([imu_data.linear_acceleration.x,
                            imu_data.linear_acceleration.y,
                            imu_data.linear_acceleration.z])

            # Predicción de la posición y velocidad usando la aceleración medida
            predicted_state = self.state.copy()
            predicted_state[0:3] += self.state[3:6] * dt + 0.5 * acc * (dt ** 2)  # Actualizar posición
            predicted_state[3:6] += acc * dt  # Actualización de la velocidad

            # Actualizar la covarianza (simplificada)
            F = np.eye(9)
            F[0:3, 3:6] = np.eye(3) * dt
            self.covariance = F @ self.covariance @ F.T + self.config.process_noise * dt

        # Corrección usando los datos GNSS
        if gnss_data:
            measurement = gnss_data['position']
            innovation = measurement - predicted_state[0:3]
            H = np.zeros((3, 9))
            H[0:3, 0:3] = np.eye(3)
            S = H @ self.covariance @ H.T + self.config.measurement_noise
            K = self.covariance @ H.T @ np.linalg.inv(S)

            # Corrección del estado y covarianza
            self.state += K @ innovation
            self.covariance = (np.eye(9) - K @ H) @ self.covariance

        return self.state
