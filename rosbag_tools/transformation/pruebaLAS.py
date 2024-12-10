import laspy
import xml.etree.ElementTree as ET
import numpy as np
from pyproj import Transformer

"""
Seguir trabajarlo
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

def merge_las_and_kml(las_file, kml_file, output_file):
    """
    Fusiona un archivo .las con coordenadas de un archivo KML.
    
    Args:
        las_file (str): Ruta al archivo .las
        kml_file (str): Ruta al archivo KML
        output_file (str): Ruta para el archivo .las de salida
    """
    # Leer el archivo .las
    las = laspy.read(las_file)
    
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
    
    # Transformar puntos del .las
    transformed_points = []
    for x, y, z in zip(las.x, las.y, las.z):
        # Agregar coordenadas locales al punto de referencia UTM
        utm_x = ref_utm_x + x
        utm_y = ref_utm_y + y
        utm_z = ref_alt + z
        
        # Convertir de vuelta a coordenadas geográficas
        lon, lat = transformer.transform(utm_x, utm_y, direction='INVERSE')
        
        transformed_points.append((lon, lat, utm_z))
    
    # Crear un nuevo archivo .las con las coordenadas transformadas
    new_las = laspy.create(
        file_version=las.header.version,
        point_format=las.header.point_format
    )
    new_las.x = np.array([point[0] for point in transformed_points])
    new_las.y = np.array([point[1] for point in transformed_points])
    new_las.z = np.array([point[2] for point in transformed_points])
    
    # Copiar otros campos si están presentes
    for dimension in las.point_format.dimension_names:
        if dimension not in ['X', 'Y', 'Z']:
            setattr(new_las, dimension, getattr(las, dimension))
    
    # Guardar el archivo .las resultante
    new_las.write(output_file)
    
    print(f"Archivo .las georeferenciado guardado en {output_file}")

# Ejemplo de uso
if __name__ == "__main__":
    # Rutas de archivos de ejemplo
    input_las = "../filesPCD/globalmap_lidar_feature.las"
    input_kml = "../filesKML/optimized_gps_trajectry.kml"
    output_las = "../filesPCD/merged_pointcloud.las"
    
    # Fusionar .las y KML
    merge_las_and_kml(input_las, input_kml, output_las)
