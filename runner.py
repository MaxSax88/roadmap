import tkinter as tk
import osmnx as ox
import networkx as nx
import folium
import numpy as np


class DrawApp:
    def __init__(self, root, process_callback):
        self.root = root
        self.root.title("Draw and Generate Route")

        self.canvas = tk.Canvas(root, width=500, height=500, bg="white")
        self.canvas.pack()

        self.points = []
        self.canvas.bind("<Button-1>", self.get_coordinates)

        self.process_callback = process_callback  # Function to process points

        # Add a button to process points and generate the map
        self.button = tk.Button(root, text="Generate Route", command=self.process_points)
        self.button.pack()

    def get_coordinates(self, event):
        """Store clicked coordinates and draw a point."""
        x, y = event.x, event.y
        self.points.append((x, y))
        self.canvas.create_oval(x - 2, y - 2, x + 2, y + 2, fill="red")

    def process_points(self):
        """Process clicked points when the button is pressed."""
        if self.points:
            self.process_callback(self.points)  # Call function with points


def get_nearest_road_nodes(graph, shape_points, map_bounds):
    """Convert clicked canvas points to lat/lon and find nearest road nodes."""
    lat_min, lat_max, lon_min, lon_max = map_bounds
    points_array = np.array(shape_points, dtype=np.float64)

    # Convert canvas (x,y) to lat/lon
    points_array[:, 1] = lon_min + (points_array[:, 0] / 500) * (lon_max - lon_min)
    points_array[:, 0] = lat_max - (points_array[:, 1] / 500) * (lat_max - lat_min)  # Flipping Y-axis

    road_nodes = []
    nearest_points = []
    for lat, lon in points_array:
        nearest_node = ox.distance.nearest_nodes(graph, lon, lat)
        road_nodes.append(nearest_node)
        nearest_points.append((lat, lon))

    return road_nodes, points_array.tolist(), nearest_points


def generate_road_route(graph, road_nodes):
    """Find the shortest path between the clicked points."""
    route = []
    for i in range(len(road_nodes) - 1):
        try:
            path = nx.shortest_path(graph, road_nodes[i], road_nodes[i + 1], weight='length')
            route.extend(path)
        except nx.NetworkXNoPath:
            continue
    return list(dict.fromkeys(route))  # Remove duplicates while preserving order


def plot_route_on_map(graph, route_nodes, original_points, nearest_points, center_latlon, output_file="route.html"):
    """Plot the route on a folium map, including original clicked points and nearest road nodes."""
    m = folium.Map(location=center_latlon, zoom_start=14)

    # Plot the original clicked points (in red)
    for lat, lon in original_points:
        folium.CircleMarker(location=[lat, lon], radius=5, color="red", fill=True, fill_color="red",
                            popup="Clicked Point").add_to(m)

    # Plot the snapped nearest road nodes (in blue)
    for lat, lon in nearest_points:
        folium.CircleMarker(location=[lat, lon], radius=5, color="blue", fill=True, fill_color="blue",
                            popup="Nearest Road Node").add_to(m)

    # Plot the road route (in green)
    route_coords = [(graph.nodes[n]['y'], graph.nodes[n]['x']) for n in route_nodes]
    folium.PolyLine(route_coords, color='green', weight=5, opacity=0.8).add_to(m)

    m.save(output_file)
    print(f"✅ Route saved as {output_file}")


def process_drawn_points(points):
    """Callback function for processing user-drawn points."""
    place_name = "Haywards Heath, UK"
    graph = ox.graph_from_place(place_name, network_type="walk")
    bounds = ox.geocode_to_gdf(place_name).total_bounds

    road_nodes, original_latlons, nearest_latlons = get_nearest_road_nodes(graph, points, bounds)

    print("Original clicked lat/lons:", original_latlons)
    print("Nearest road nodes lat/lons:", nearest_latlons)

    route = generate_road_route(graph, road_nodes)

    # Compute map center
    map_center = ((bounds[1] + bounds[3]) / 2, (bounds[0] + bounds[2]) / 2)
    plot_route_on_map(graph, route, original_latlons, nearest_latlons, map_center)


if __name__ == "__main__":
    root = tk.Tk()
    app = DrawApp(root, process_callback=process_drawn_points)
    root.mainloop()  # Keeps Tkinter open until user closes it
