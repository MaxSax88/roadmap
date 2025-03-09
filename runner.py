import osmnx as ox
import networkx as nx
import numpy as np
import cv2
import folium
from shapely.geometry import LineString, Point


# ============================
# 1️⃣ Load and Process Input Shape
# ============================

def extract_shape_outline(image_path, resize_dim=(500, 500)):
    """
    Extracts the contour (outline) of a shape from an input image.

    Args:
        image_path (str): Path to the input image.
        resize_dim (tuple): New dimensions for the image.

    Returns:
        list: List of (x, y) points representing the shape's contour.
    """
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    image = cv2.resize(image, resize_dim)

    _, thresh = cv2.threshold(image, 128, 255, cv2.THRESH_BINARY_INV)  # Invert colors
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    # Find the largest contour (assumes it's the shape)
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        shape_points = [(pt[0][0], pt[0][1]) for pt in contours]
        print(shape_points)
        return shape_points
    return []


# ============================
# 2️⃣ Match Shape to Road Network
# ============================

def get_nearest_road_nodes(graph, shape_points, map_bounds):
    """
    Maps shape contour points to the nearest road network nodes.

    Args:
        graph: Road network graph from OSM.
        shape_points: List of (x, y) points representing the shape.
        map_bounds: Bounding box of the area (lat_min, lat_max, lon_min, lon_max).

    Returns:
        list: List of nearest road nodes that approximate the shape.
    """
    lat_min, lat_max, lon_min, lon_max = map_bounds

    # Normalize shape points to the bounding box
    shape_points = np.array(shape_points)
    shape_points[:, 0] = lat_min + (shape_points[:, 0] / 500) * (lat_max - lat_min)
    shape_points[:, 1] = lon_min + (shape_points[:, 1] / 500) * (lon_max - lon_min)

    # Find nearest road nodes for each shape point
    road_nodes = []
    for lat, lon in shape_points:
        nearest_node = ox.distance.nearest_nodes(graph, lon, lat)
        road_nodes.append(nearest_node)

    return road_nodes


# ============================
# 3️⃣ Generate a Route Following Roads
# ============================

def generate_road_route(graph, road_nodes):
    """
    Finds the best road-based route that follows the mapped shape.

    Args:
        graph: Road network graph from OSM.
        road_nodes: List of mapped road network nodes.

    Returns:
        list: List of road segments forming the route.
    """
    route = []
    for i in range(len(road_nodes) - 1):
        try:
            path = nx.shortest_path(graph, road_nodes[i], road_nodes[i + 1], weight='length')
            route.extend(path)
        except nx.NetworkXNoPath:
            continue  # Skip if no path exists between nodes

    return list(dict.fromkeys(route))  # Remove duplicates


# ============================
# 4️⃣ Plot the Final Route on a Map
# ============================

def plot_route_on_map(graph, route_nodes, center_latlon):
    """
    Plots the generated road-based route on an interactive map.

    Args:
        graph: Road network graph.
        route_nodes: List of nodes forming the route.
        center_latlon: Tuple of (latitude, longitude) for map center.
    """
    m = folium.Map(location=center_latlon, zoom_start=14)

    # Convert nodes to coordinates
    route_coords = [(graph.nodes[n]['y'], graph.nodes[n]['x']) for n in route_nodes]

    # Plot the route
    folium.PolyLine(route_coords, color='blue', weight=5, opacity=0.8).add_to(m)

    # Show the map
    return m


# ============================
# 5️⃣ Main Execution
# ============================

def main():
    # 1️⃣ Extract shape points from input image
    shape_image = r"C:\Users\mgbsa\Desktop\sauqre.jpg"  # Replace with your input shape image
    shape_points = extract_shape_outline(shape_image)

    if not shape_points:
        print("❌ No shape detected!")
        return

    # 2️⃣ Define map location (e.g., central London)
    place_name = "Edinburgh, UK"
    graph = ox.graph_from_place(place_name, network_type="drive")

    # Get bounding box of the area
    bounds = ox.geocode_to_gdf(place_name).total_bounds  # (lon_min, lat_min, lon_max, lat_max)

    # 3️⃣ Map shape to road network
    road_nodes = get_nearest_road_nodes(graph, shape_points, bounds)

    # 4️⃣ Generate the road-following route
    route_nodes = generate_road_route(graph, road_nodes)

    if not route_nodes:
        print("❌ No valid route found!")
        return

    # 5️⃣ Display the route on a map
    map_center = (bounds[1] + bounds[3]) / 2, (bounds[0] + bounds[2]) / 2  # Average center
    route_map = plot_route_on_map(graph, route_nodes, map_center)

    # Save and display map
    route_map.save("route_map.html")
    print("✅ Route generated! Open 'route_map.html' in a browser.")


if __name__ == "__main__":
    main()
