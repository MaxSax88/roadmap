import osmnx as ox
import networkx as nx
import folium
import numpy as np
from shapely.geometry import LineString, Point
from shapely import affinity
import cv2
import matplotlib.pyplot as plt
from shapely.geometry import MultiPoint
from alphashape import alphashape


def extract_shape_outline(image_path, resize_dim=(50, 50),min_area=2,alpha=0.05):
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    image = cv2.resize(image, resize_dim)

    _, thresh = cv2.threshold(image, 80, 255, cv2.THRESH_BINARY_INV)
    cannyimg = cv2.Canny(thresh, 100, 200)
    contours, _ = cv2.findContours(cannyimg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    filtered_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]
    all_points = np.vstack(filtered_contours)
    alpha_shape = alphashape(all_points, alpha)
    if alpha_shape.geom_type == "Polygon":
        contour_points = np.array(alpha_shape.exterior.coords)
        shape_points = contour_points.tolist()

    plt.scatter([x[0] for x in shape_points], [x[1] for x in shape_points])
    plt.show()
    print(len(shape_points))
    return shape_points


def transform_shape(shape_points, angle, dx, dy):
    """Applies rotation and translation to shape points."""
    shape = LineString(shape_points)
    rotated = affinity.rotate(shape, angle, origin='center')
    translated = affinity.translate(rotated, xoff=dx, yoff=dy)
    return list(translated.coords)


def get_nearest_road_nodes(graph, shape_points, map_bounds):
    lat_min, lat_max, lon_min, lon_max = map_bounds
    shape_points = np.array(shape_points)
    shape_points[:, 0] = lat_min + (shape_points[:, 0] / 500) * (lat_max - lat_min)
    shape_points[:, 1] = lon_min + (shape_points[:, 1] / 500) * (lon_max - lon_min)

    road_nodes = []
    for lat, lon in shape_points:
        nearest_node = ox.distance.nearest_nodes(graph, lon, lat)
        road_nodes.append(nearest_node)
    return road_nodes


def generate_road_route(graph, road_nodes):
    route = []
    for i in range(len(road_nodes) - 1):
        try:
            path = nx.shortest_path(graph, road_nodes[i], road_nodes[i + 1], weight='length')
            route.extend(path)
        except nx.NetworkXNoPath:
            continue
    return list(dict.fromkeys(route))


def evaluate_fit(graph, road_nodes):
    """Evaluates how well the shape fits onto the road network."""
    return sum(nx.shortest_path_length(graph, road_nodes[i], road_nodes[i + 1], weight='length') for i in
               range(len(road_nodes) - 1))


def find_best_shape_fit(graph, shape_points, bounds):
    """Searches different rotations and translations to find the best shape fit."""
    best_route = None
    best_score = float('inf')
    best_params = None

    for angle in range(0, 360, 30):  # Rotate in 30-degree increments
        for dx in np.linspace(-0.01, 0.01, 5):  # Small x-shift adjustments
            for dy in np.linspace(-0.01, 0.01, 5):  # Small y-shift adjustments
                transformed_shape = transform_shape(shape_points, angle, dx, dy)
                road_nodes = get_nearest_road_nodes(graph, transformed_shape, bounds)
                if len(road_nodes) > 1:
                    route = generate_road_route(graph, road_nodes)
                    score = evaluate_fit(graph, road_nodes)
                    if score < best_score:
                        best_score = score
                        best_route = route
                        best_params = (angle, dx, dy)

    print(f"Best Fit: Rotation {best_params[0]} degrees, Translation ({best_params[1]}, {best_params[2]})")
    return best_route


def plot_route_on_map(graph, route_nodes, center_latlon, output_file="elephant_route.html"):
    m = folium.Map(location=center_latlon, zoom_start=14)
    route_coords = [(graph.nodes[n]['y'], graph.nodes[n]['x']) for n in route_nodes]
    folium.PolyLine(route_coords, color='blue', weight=5, opacity=0.8).add_to(m)
    m.save(output_file)
    print(f"✅ Route saved as {output_file}")


def main():
    #shape_image = r"C:\Users\mgbsa\Desktop\sauqre.jpg"  # Replace with your elephant outline image
    shape_image = r"C:\Users\mgbsa\Desktop\baby-elephant-outline.jpg"
    shape_points = extract_shape_outline(shape_image)

    if not shape_points:
        print("❌ No shape detected!")
        return

    place_name = "Edinburgh, Scotland"
    graph = ox.graph_from_place(place_name, network_type="walk")
    bounds = ox.geocode_to_gdf(place_name).total_bounds

    best_route = find_best_shape_fit(graph, shape_points, bounds)

    if not best_route:
        print("❌ No valid route found!")
        return

    map_center = ((bounds[1] + bounds[3]) / 2, (bounds[0] + bounds[2]) / 2)
    plot_route_on_map(graph, best_route, map_center)


if __name__ == "__main__":
    main()
