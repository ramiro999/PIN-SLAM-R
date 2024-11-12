import csv
from simplekml import Kml

# Crear un objeto KML
kml = Kml()

# Leer los datos del archivo CSV
with open('./output_gps/output.csv', 'r') as csvfile:
    reader = csv.DictReader(csvfile)
    
    # Iterar sobre cada línea del CSV y agregar los puntos al KML
    for row in reader:
        x = float(row['X'])
        y = float(row['Y'])
        z = float(row['Z'])
        
        # Crear un nuevo punto en el KML
        kml.newpoint(name="", coords=[(x, y, z)])  # Se usa (longitud, latitud, altura)

# Guardar el archivo KML
kml.save("./output_gps/output.kml")
