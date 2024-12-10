import open3d as o3d
import xml.etree.ElementTree as ET
import numpy as np
from pyproj import Transformer
import utm

"""
Este script es la segunda prueba para la transformación de una nube de puntos PCD con coordenadas geográficas a un sistema de coordenadas diferente.
"""

def read_kml_coordinates(kml_file):
    """
    Extrae las coordenadas del archivo KML.
    
    Args:
        kml_file (str): Ruta al archivo KML
    
    Returns:
        list: Lista de coordenadas (longitud, latitud, altura)
    """
    tree = ET.parse(kml_file)
    root = tree.getroot()
    
    # Definir el namespace de KML
    namespace = {'kml': 'http://www.opengis.net/kml/2.2'}
    
    # Encontrar todos los elementos de coordenadas
    coords_elements = root.findall(".//kml:coordinates", namespace)
    
    coordinates = []
    for coords_element in coords_elements:
        coords_text = coords_element.text.strip()
        for line in coords_text.split():
            lon, lat, alt = map(float, line.split(","))
            coordinates.append((lon, lat, alt))
    
    return coordinates

def merge_pcd_and_kml(pcd_file, kml_file, output_file):
    """
    Fusiona una nube de puntos PCD con coordenadas de un archivo KML.
    
    Args:
        pcd_file (str): Ruta al archivo PCD
        kml_file (str): Ruta al archivo KML
        output_file (str): Ruta para el archivo PCD de salida
    """
    # Leer la nube de puntos
    point_cloud = o3d.io.read_point_cloud(pcd_file)
    points = np.asarray(point_cloud.points)
    
    # Leer coordenadas del KML
    kml_coords = read_kml_coordinates(kml_file)
    
    # Verificar que hay suficientes coordenadas KML
    if not kml_coords:
        raise ValueError("No se encontraron coordenadas en el archivo KML")
    
    # Usar la primera coordenada del KML como punto de referencia
    ref_lon, ref_lat, ref_alt = kml_coords[0]
    
    # Crear transformador para convertir coordenadas
    transformer = Transformer.from_crs("EPSG:4326", "EPSG:32619", always_xy=True)
    
    # Convertir punto de referencia a coordenadas UTM
    ref_utm_x, ref_utm_y = transformer.transform(ref_lon, ref_lat)
    
    # Transformar puntos de la nube
    transformed_points = []
    for point in points:
        # Agregar coordenadas locales al punto de referencia UTM
        utm_x = ref_utm_x + point[0]
        utm_y = ref_utm_y + point[1]
        utm_z = ref_alt + point[2]
        
        # Convertir de vuelta a coordenadas geográficas
        lon, lat = transformer.transform(utm_x, utm_y, direction='INVERSE')
        
        transformed_points.append([lon, lat, utm_z])
    
    # Crear nueva nube de puntos
    merged_cloud = o3d.geometry.PointCloud()
    merged_cloud.points = o3d.utility.Vector3dVector(np.array(transformed_points))
    
    # Copiar colores si existen en la nube original
    if point_cloud.has_colors():
        merged_cloud.colors = point_cloud.colors
    
    # Guardar la nube de puntos fusionada
    o3d.io.write_point_cloud(output_file, merged_cloud, write_ascii=True)
    
    print(f"Nube de puntos fusionada guardada en {output_file}")
    print(f"Total de puntos: {len(transformed_points)}")

def create_kml_from_pointcloud(pcd_file, output_kml):
    """
    Crea un archivo KML a partir de una nube de puntos con coordenadas geográficas.
    
    Args:
        pcd_file (str): Ruta al archivo PCD con coordenadas geográficas
        output_kml (str): Ruta para el archivo KML de salida
    """
    # Leer la nube de puntos
    point_cloud = o3d.io.read_point_cloud(pcd_file)
    points = np.asarray(point_cloud.points)
    
    # Crear la estructura XML del KML
    kml = ET.Element('kml', {'xmlns': 'http://www.opengis.net/kml/2.2'})
    document = ET.SubElement(kml, 'Document')
    
    # Crear un placemark para cada punto
    for i, point in enumerate(points):
        placemark = ET.SubElement(document, 'Placemark')
        ET.SubElement(placemark, 'name').text = f'Point {i+1}'
        
        # Formatear coordenadas para KML
        coords_text = f"{point[0]},{point[1]},{point[2]}"
        ET.SubElement(placemark, 'Point').text = coords_text
    
    # Crear el árbol XML
    tree = ET.ElementTree(kml)
    
    # Guardar el archivo KML
    tree.write(output_kml, encoding='utf-8', xml_declaration=True)
    
    print(f"Archivo KML creado en {output_kml}")

# Ejemplo de uso
if __name__ == "__main__":
    # Rutas de archivos de ejemplo
    input_pcd = "../filesPCD/globalmap_lidar_feature.pcd"
    input_kml = "../filesKML/optimized_gps_trajectry.kml"
    output_pcd = "../filesPCD/result2_pointcloud.pcd"
    output_kml = "../filesKML/output_pointcloud.kml"
    
    # Fusionar PCD y KML
    merge_pcd_and_kml(input_pcd, input_kml, output_pcd)
    
    # Opcionalmente, crear KML a partir de la nube de puntos
    create_kml_from_pointcloud(output_pcd, output_kml)