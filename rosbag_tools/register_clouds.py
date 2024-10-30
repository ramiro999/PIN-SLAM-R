import numpy as np
import probreg
import open3d as o3d
from probreg import icp

def load_point_cloud(file_path):
    """Carga una nube de puntos de un archivo PLY o PCD"""
    pcd = o3d.io.read_point_cloud(file_path)
    return pcd

def register_point_clouds(source_path, target_path):
    """Realiza el registro de dos nubes de puntos usando ICP"""
    source = load_point_cloud(source_path)
    target = load_point_cloud(target_path)

    # Convertir las nubes a numpy arrays para el registro
    source_np = np.asarray(source.points)
    target_np = np.asarray(target.points)

    # Realizar el registro ICP
    tf_param, _, _ = icp.registration_icp(source_np, target_np)

    # Aplicar la transformación a la nube de origen
    source.transform(tf_param.transformation)

    # Visualizar la nube transformada y la original
    o3d.visualization.draw_geometries([source, target])

if __name__ == "__main__":
    # Reemplaza las rutas con tus archivos PLY/PCD
    source_path = "source.ply"
    target_path = "target.ply"
    register_point_clouds(source_path, target_path)
