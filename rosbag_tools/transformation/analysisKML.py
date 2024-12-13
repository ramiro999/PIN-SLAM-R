import xml.etree.ElementTree as ET
import pandas as pd
import numpy as np
from typing import List, Tuple, Dict, Union
import xmltodict
import json
from colorama import init, Fore, Style
import math

# Initialize colorama for colored output
init(autoreset=True)

class KMLParser:
    def __init__(self, file_path: str):
        """
        Initialize the KML parser with a file path
        
        :param file_path: Path to the KML file
        """
        self.file_path = file_path
        self.tree = ET.parse(file_path)
        self.root = self.tree.getroot()
        
        # Define the namespace to handle KML XML correctly
        self.namespace = {
            'kml': 'http://www.opengis.net/kml/2.2'
        }
    
    def extract_coordinates(self, format: str = 'tuple') -> Union[List[Tuple[float, float, float]], List[Dict], str]:
        """
        Extract coordinates from the KML file
        
        :param format: Output format. Options:
            - 'tuple': List of (longitude, latitude, altitude) tuples (default)
            - 'dict': List of dictionaries with lon, lat, alt keys
            - 'pretty': Formatted string for easy reading
            - 'csv': CSV-style string of coordinates
        :return: Coordinates in specified format
        """
        # Find all coordinate elements
        coord_elements = self.root.findall('.//kml:coordinates', namespaces=self.namespace)
        
        coordinates = []
        for coord_elem in coord_elements:
            # Split coordinates and convert to float tuples
            coord_list = coord_elem.text.strip().split('\n')
            for coord in coord_list:
                if coord.strip():
                    lon, lat, alt = map(float, coord.strip().split(','))
                    
                    # Convert to desired format
                    if format == 'tuple':
                        coordinates.append((lon, lat, alt))
                    elif format == 'dict':
                        coordinates.append({
                            'longitude': lon, 
                            'latitude': lat, 
                            'altitude': alt
                        })
        
        # Return formatted output
        if format == 'tuple':
            return coordinates
        elif format == 'dict':
            return coordinates
        elif format == 'pretty':
            return '\n'.join([
                f"{Fore.GREEN}Coordinate:{Fore.RESET} Longitude: {Fore.BLUE}{lon:.6f}{Fore.RESET}, "
                f"Latitude: {Fore.BLUE}{lat:.6f}{Fore.RESET}, Altitude: {Fore.BLUE}{alt:.2f}{Fore.RESET}"
                for lon, lat, alt in coordinates
            ])
        elif format == 'csv':
            return '\n'.join([
                f"{lon:.6f},{lat:.6f},{alt:.2f}"
                for lon, lat, alt in coordinates
            ])
        
        return coordinates
    
    def calculate_total_distance(self) -> float:
        """
        Calculate total distance traveled based on coordinates
        Uses Haversine formula for distance calculation
        
        :return: Total distance in kilometers
        """
        def haversine_distance(lon1, lat1, lon2, lat2):
            """Calculate distance between two points on earth"""
            R = 6371  # Earth's radius in kilometers
            
            # Convert latitude and longitude to radians
            lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])
            
            # Haversine formula
            dlon = lon2 - lon1
            dlat = lat2 - lat1
            a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
            c = 2 * math.asin(math.sqrt(a))
            
            return R * c
        
        coords = self.extract_coordinates()
        
        total_distance = 0
        for i in range(1, len(coords)):
            lon1, lat1, _ = coords[i-1]
            lon2, lat2, _ = coords[i]
            total_distance += haversine_distance(lon1, lat1, lon2, lat2)
        
        return total_distance
    
    def extract_styles(self) -> Dict[str, Dict]:
        """
        Extract style information from the KML file
        
        :return: Dictionary of style details
        """
        styles = {}
        for style in self.root.findall('.//kml:Style', namespaces=self.namespace):
            style_id = style.get('id')
            style_details = {}
            
            # Extract LineStyle
            line_style = style.find('.//kml:LineStyle', namespaces=self.namespace)
            if line_style is not None:
                color = line_style.find('kml:color', namespaces=self.namespace)
                width = line_style.find('kml:width', namespaces=self.namespace)
                
                style_details['line_color'] = color.text if color is not None else None
                style_details['line_width'] = float(width.text) if width is not None else None
            
            # Extract PolyStyle
            poly_style = style.find('.//kml:PolyStyle', namespaces=self.namespace)
            if poly_style is not None:
                color = poly_style.find('kml:color', namespaces=self.namespace)
                style_details['poly_color'] = color.text if color is not None else None
            
            styles[style_id] = style_details
        
        return styles
    
    def extract_placemarks(self) -> List[Dict]:
        """
        Extract detailed information about placemarks
        
        :return: List of placemark details
        """
        placemarks = []
        for placemark in self.root.findall('.//kml:Placemark', namespaces=self.namespace):
            placemark_info = {}
            
            # Extract name
            name = placemark.find('kml:name', namespaces=self.namespace)
            placemark_info['name'] = name.text if name is not None else None
            
            # Extract description
            description = placemark.find('kml:description', namespaces=self.namespace)
            placemark_info['description'] = description.text if description is not None else None
            
            # Extract style URL
            style_url = placemark.find('kml:styleUrl', namespaces=self.namespace)
            placemark_info['style_url'] = style_url.text if style_url is not None else None
            
            # Extract coordinates
            coords_elem = placemark.find('.//kml:coordinates', namespaces=self.namespace)
            if coords_elem is not None:
                coords = [
                    tuple(map(float, coord.strip().split(','))) 
                    for coord in coords_elem.text.strip().split('\n') 
                    if coord.strip()
                ]
                placemark_info['coordinates'] = coords
            
            placemarks.append(placemark_info)
        
        return placemarks
    
    def convert_coordinates_to_dataframe(self) -> pd.DataFrame:
        """
        Convert extracted coordinates to a pandas DataFrame
        
        :return: DataFrame with longitude, latitude, and altitude
        """
        coords = self.extract_coordinates()
        df = pd.DataFrame(coords, columns=['longitude', 'latitude', 'altitude'])
        return df
    
    def to_dict(self) -> Dict:
        """
        Convert the entire KML file to a dictionary
        
        :return: Dictionary representation of the KML file
        """
        with open(self.file_path, 'r') as file:
            return xmltodict.parse(file.read())
    
    def to_json(self, indent: int = 2) -> str:
        """
        Convert the KML file to a JSON string
        
        :param indent: Number of spaces for indentation
        :return: JSON string representation of the KML file
        """
        return json.dumps(self.to_dict(), indent=indent)

# Example usage
def main():
    try:
        kml_parser = KMLParser('../filesKML/optimized_gps_trajectry.kml')
        
        # Extract and display coordinates in different formats
        print(f"\n{Fore.CYAN}===== Coordinates (Pretty Format) ====={Fore.RESET}")
        print(kml_parser.extract_coordinates(format='pretty')[:10])  # First 10 coordinates
        
        # Convert coordinates to DataFrame
        print(f"\n{Fore.CYAN}===== Coordinates DataFrame (First 5 Rows) ====={Fore.RESET}")
        df = kml_parser.convert_coordinates_to_dataframe()
        print(df.head())
        
        # Calculate total distance
        total_distance = kml_parser.calculate_total_distance()
        print(f"\n{Fore.YELLOW}Total Distance Traveled: {Fore.GREEN}{total_distance:.2f} km{Fore.RESET}")
        
        # Extract and display styles
        print(f"\n{Fore.CYAN}===== Styles ====={Fore.RESET}")
        styles = kml_parser.extract_styles()
        print(json.dumps(styles, indent=2))
        
        # Extract and display placemarks
        print(f"\n{Fore.CYAN}===== Placemarks ====={Fore.RESET}")
        placemarks = kml_parser.extract_placemarks()
        print(json.dumps(placemarks, indent=2))
    
    except FileNotFoundError:
        print(f"{Fore.RED}Error: KML file not found. Please check the file path.{Fore.RESET}")
    except ET.ParseError:
        print(f"{Fore.RED}Error: Unable to parse the KML file. Ensure it's a valid KML format.{Fore.RESET}")
    except Exception as e:
        print(f"{Fore.RED}An unexpected error occurred: {e}{Fore.RESET}")

if __name__ == '__main__':
    main()

# Required dependencies:
# pip install lxml pandas xmltodict colorama numpy