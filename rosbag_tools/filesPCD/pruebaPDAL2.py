import pdal
import json  # Importar el módulo json

# Definir el pipeline inicial para extraer bbox
pipeline_extract_json = """
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
        "type": "filters.reprojection",
        "in_srs": "EPSG:4326",
        "out_srs": "EPSG:32619"
    },
    {
        "type": "writers.las",
        "filename": "globalmap_lidar_feature.las",
        "a_srs": "EPSG:32619"
    }
]
"""

# Ejecutar el pipeline inicial
pipeline = pdal.Pipeline(pipeline_extract_json)
try:
    count = pipeline.execute()
    print(f"Pipeline ejecutado con éxito. {count} puntos procesados.")
except RuntimeError as e:
    print(f"Error ejecutando el pipeline: {e}")

# Obtener metadatos directamente (ya es un dict)
metadata = pipeline.metadata

# Imprimir la estructura completa de los metadatos para inspección
print(json.dumps(metadata, indent=4))
