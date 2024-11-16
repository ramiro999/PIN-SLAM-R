import laspy
import pandas as pd
"""
Este script genera un archivo CSV a partir de un archivo .las georreferenciado con WGS84.
"""

# Load the .las file
las_file_path = '../lasFiles/points.las'  # Update with your .las file path
las_data = laspy.open(las_file_path).read()


# Extract data from the .las file
points_data = {
    "X": las_data.X * las_data.header.scale[0] + las_data.header.offset[0],
    "Y": las_data.Y * las_data.header.scale[1] + las_data.header.offset[1],
    "Z": las_data.Z * las_data.header.scale[2] + las_data.header.offset[2],
}

# Convert to a DataFrame
df = pd.DataFrame(points_data)

# Save to a .csv file
csv_file_path = '../outputs/outputs_las2csv/points.csv'  # Desired .csv output path
df.to_csv(csv_file_path, index=False)

print(f"CSV file saved at: {csv_file_path}")
