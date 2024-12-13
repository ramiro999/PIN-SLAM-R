import pdal

# Definimos el pipeline de PDAL en formato JSON
pipeline_json = """
[
    {
        "type": "readers.pcd",
        "filename": "globalmap_lidar_feature.pcd",
        "override_srs": "EPSG:32619"
    },
    {
        "type": "filters.reprojection",
        "in_srs": "EPSG:32619",
        "out_srs": "EPSG:4326"
    },
    {
        "type": "filters.transformation",
        "matrix": "1  0  0  73.49028837 0  1  0  0.007017198786  0  0  1  100.62806  0  0  0  1"
    },
    {
        "type": "filters.transformation",
        "matrix": "1  0  0  114.263613010405  0  1  0  22.3387157880116  0  0  1  96.0567769246592  0  0  0  1"
    },
    {
        "type": "writers.las",
        "filename": "globalmap_lidar_feature.las",
        "a_srs": "EPSG:4326",
        "scale_x": "0.0000001",
        "scale_y": "0.0000001",
        "scale_z": "0.0000001",
        "offset_x": "114.263613010405",
        "offset_y": "22.3387157880116",
        "offset_z": "96.0567769246592"
    }
]
"""

# Cargar el pipeline en PDAL
pipeline = pdal.Pipeline(pipeline_json)

# Ejecutar el pipeline
try:
    count = pipeline.execute()
    print(f"Pipeline ejecutado con éxito. {count} puntos procesados.")
except RuntimeError as e:
    print(f"Error ejecutando el pipeline: {e}")

# Obtener metadatos y log del pipeline
metadata = pipeline.metadata
log = pipeline.log

# Opcional: Imprimir metadatos y log
print("Metadatos del pipeline:")
print(metadata)

print("\nLog del pipeline:")
print(log)
