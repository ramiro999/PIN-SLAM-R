import laspy
import simplekml
"""
Este script convierte un archivo .las en un archivo .kml con los puntos georreferenciados.
"""

# Ruta del archivo .las
las_file_path = '../lasFiles/points.las' 
kml_file_path = '../outputs/outputs_las2kml/points.kml'  # Ruta del archivo .kml de salida

# Leer el archivo .las
with laspy.open(las_file_path) as las:
    las_data = las.read()

# Crear un archivo KML
kml = simplekml.Kml()

# Extraer y agregar puntos al archivo KML
scale_x, scale_y, scale_z = las.header.scale
offset_x, offset_y, offset_z = las.header.offset

for x, y, z in zip(
    las_data.X * scale_x + offset_x,
    las_data.Y * scale_y + offset_y,
    las_data.Z * scale_z + offset_z
):
    kml.newpoint(coords=[(x, y, z)])  # Coordenadas en formato (lon, lat, alt)

# Guardar el archivo KML
kml.save(kml_file_path)

print(f"Archivo KML generado en: {kml_file_path}")
