import pandas as pd
"""
Este script procesa un archivo CSV con datos GPS del topico de gps del rosbag y genera un archivo KML con los puntos GPS.
"""

# Ruta del archivo CSV
csv_path = './output_gps/gps_data.csv'

# Recargar el archivo CSV proporcionado
gps_data = pd.read_csv(csv_path)

# Crear un archivo KML manualmente usando los datos del CSV

# Crear un archivo KML vacío
kml_content = """<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>GPS Data from ROSBag</name>
"""

# Iterar sobre el DataFrame para agregar cada punto GPS al KML
for index, row in gps_data.iterrows():
    latitude = row['latitude']
    longitude = row['longitude']
    altitude = row['altitude']
    kml_content += f"""
    <Placemark>
      <name>Point {index+1}</name>
      <Point>
        <coordinates>{longitude},{latitude},{altitude}</coordinates>
      </Point>
    </Placemark>
    """

# Cerrar las etiquetas del KML
kml_content += """
  </Document>
</kml>
"""

# Guardar el contenido en un archivo .kml
output_kml_path = "./output_gps/gps_data.kml"
with open(output_kml_path, "w") as kml_file:
    kml_file.write(kml_content)

# Indicar la ubicación del archivo KML generado
output_kml_path
