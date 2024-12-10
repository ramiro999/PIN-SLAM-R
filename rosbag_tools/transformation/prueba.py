import open3d as o3d
import xml.etree.ElementTree as ET
import numpy as np
from pyproj import Transformer

def read_kml_coordinates(kml_file):
    """Extrae las coordenadas del archivo KML."""
    tree = ET.parse(kml_file)
    root = tree.getroot()

    # Buscar las coordenadas en el nodo <coordinates>
    namespace = {'kml': 'http://www.opengis.net/kml/2.2'}
    coords_text = root.find(".//kml:coordinates", namespace).text.strip()

    # Convertir las coordenadas a una lista de (longitud, latitud, altitud)
    coordinates = []
    for line in coords_text.split():
        lon, lat, alt = map(float, line.split(","))
        coordinates.append((lon, lat, alt))
    return coordinates

def transform_pcd_to_georeferenced(pcd_file, kml_file, output_pcd):
    """Transforma un archivo PCD sin georreferencia utilizando un KML."""
    # Leer las coordenadas del KML
    kml_coords = read_kml_coordinates(kml_file)
    ref_lon, ref_lat, ref_alt = kml_coords[0]  # Usamos el primer punto del KML como referencia
    print(f"Punto de referencia (KML): Longitud={ref_lon}, Latitud={ref_lat}, Altitud={ref_alt}")

    # Leer la nube de puntos del PCD
    cloud = o3d.io.read_point_cloud(pcd_file)
    points = np.asarray(cloud.points)

    # Configurar transformación con PyProj
    transformer = Transformer.from_crs("epsg:4978", "epsg:4326", always_xy=True)  # Geocéntrico a WGS84

    # Transformar puntos
    transformed_points = []
    for point in points:
        x, y, z = point
        lon, lat, alt = transformer.transform(x, y, z)
        transformed_points.append([lon, lat, alt])

    # Crear una nueva nube de puntos con las coordenadas transformadas
    transformed_cloud = o3d.geometry.PointCloud()
    transformed_cloud.points = o3d.utility.Vector3dVector(np.array(transformed_points))

    # Guardar la nube de puntos en formato ASCII
    o3d.io.write_point_cloud(output_pcd, transformed_cloud, write_ascii=True)
    print(f"Nube de puntos georreferenciada guardada en {output_pcd}")

if __name__ == "__main__":
    # Archivos de entrada y salida
    input_pcd = "../filesPCD/globalmap_lidar_feature.pcd"
    input_kml = "../filesKML/optimized_gps_trajectry.kml"
    output_pcd = "../filesPCD/georeferenced_output.pcd"

    # Transformar la nube de puntos
    transform_pcd_to_georeferenced(input_pcd, input_kml, output_pcd)
