import lasio
import pandas as pd

"""
Este script procesa varios archivos CSV de point_cloud, imu_data y gps_data y genera un archivo LAS con las curvas de cada archivo CSV.

No esta funcionando como debería solo se estan guardando la primera fila de cada archivo csv que contiene todas las columnas solmanente
"""

# List of CSV files to convert
csv_files = ['./data-csv-las/point-cloud-data.csv', './data-csv-las/imu-data.csv', './data-csv-las/gps-data.csv']

# Create an empty LAS file
las = lasio.LASFile()

# Iterate over each CSV file
for csv_file in csv_files:
    # Read the CSV file into a pandas DataFrame
    df = pd.read_csv(csv_file)

    # Add each column in the DataFrame as a separate curve to the LAS file
    for column in df.columns:
        # Adding a curve for each column in the DataFrame
        las.curves.append(lasio.CurveItem(mnemonic=column, data=df[column].values, unit="unit", descr="description"))

# Save the LAS file
las.write('./data-csv-las/output.las')

# Cargar el archivo .las generado
las = lasio.read('./data-csv-las/output.las')

# Imprimir las curvas y sus detalles
for curve in las.curves:
    print(f"Curva: {curve.mnemonic}")
    print(f"Unidad: {curve.unit}")
    print(f"Descripción: {curve.descr}")
    print(f"Primeros 5 datos de la curva: {curve.data[:5]}")
    print()

# Convert las file to csv
df = las.df()
df.to_csv('./data-csv-las/output.csv')