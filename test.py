import tkinter as tk
import osmnx as ox
import networkx as nx
import folium
import numpy as np
from shapely.geometry import LineString
from shapely import affinity



class DrawApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Click to Get Coordinates")

        self.canvas = tk.Canvas(root, width=500, height=500, bg="white")
        self.canvas.pack()

        self.points = []
        self.canvas.bind("<Button-1>", self.get_coordinates)

    def get_coordinates(self, event):
        x, y = event.x, event.y
        self.points.append((x, y))
        self.canvas.create_oval(x-2, y-2, x+2, y+2, fill="red")


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





if __name__ == "__main__":
    root = tk.Tk()
    app = DrawApp(root)
    root.mainloop()
    coordinates = app.points
    place_name = "Edinburgh, Scotland"
    graph = ox.graph_from_place(place_name, network_type="walk")
    bounds = ox.geocode_to_gdf(place_name).total_bounds

    best_route = find_best_shape_fit(graph, coordinates, bounds)

    map_center = ((bounds[1] + bounds[3]) / 2, (bounds[0] + bounds[2]) / 2)
    plot_route_on_map(graph, best_route, map_center)