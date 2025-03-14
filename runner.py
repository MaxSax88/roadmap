import tkinter as tk
import osmnx as ox
import networkx as nx
import folium
import numpy as np
from get_lat_lon import get_lat_lon


class DrawApp:
    def __init__(self, root, process_callback):
        self.root = root
        self.root.title("Draw and Generate Route")

        self.canvas = tk.Canvas(root, width=500, height=500, bg="white")
        self.canvas.pack()

        self.points = []
        self.canvas.bind("<Button-1>", self.get_coordinates)

        self.process_callback = process_callback  # Function to process points

        # Distance input field
        self.label = tk.Label(root, text="Target Distance (km):")
        self.label.pack()
        self.distance_entry = tk.Entry(root)
        self.distance_entry.pack()

        # Postcode input field
        self.label = tk.Label(root, text="Postcode:")
        self.label.pack()
        self.postcode = tk.Entry(root)
        self.postcode.pack()

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
        self.root.destroy()



def get_nearest_road_nodes(graph, shape_points, map_bounds):
    """Convert clicked canvas points to lat/lon and find nearest road nodes."""
    x_min, y_min, x_max, y_max = map_bounds
    points_array = np.array(shape_points, dtype=np.float64)
    points_array[:, 0] = x_min + (points_array[:, 0] / 500) * (x_max - x_min)
    points_array[:, 1] = y_min + (points_array[:, 1] / 500) * (y_max - y_min)
    road_nodes = []
    for x, y in points_array:
        nearest_node = ox.distance.nearest_nodes(graph, x, y)
        road_nodes.append(nearest_node)
    return road_nodes


def generate_road_route(graph, road_nodes):
    """Find the shortest path between the clicked points."""
    route = []
    distance = 0
    for i in range(len(road_nodes) - 1):
        try:
            path = nx.shortest_path(graph, road_nodes[i], road_nodes[i + 1], weight='length')
            route.extend(path)
            distance += nx.shortest_path_length(graph, road_nodes[i], road_nodes[i+1], weight="length")
        except nx.NetworkXNoPath:
            continue
    last_path = nx.shortest_path(graph, road_nodes[-1], road_nodes[0], weight='length')
    route.extend(last_path)
    print(f'The route I found you is {distance/1000:.0f}km long. Happy tracks!')
    return list(dict.fromkeys(route))  # Remove duplicates while preserving order


def plot_route_on_map(graph, route_nodes, center_latlon, output_file="route.html"):
    """Plot the route on a folium map, including original clicked points and nearest road nodes."""
    m = folium.Map(location=center_latlon, zoom_start=14)

    # Plot the road route (in green)
    route_coords = [(graph.nodes[n]['y'], graph.nodes[n]['x']) for n in route_nodes]
    folium.PolyLine(route_coords, color='green', weight=5, opacity=0.8).add_to(m)

    m.save(output_file)
    print(f"✅ Route saved as {output_file}")


def process_drawn_points(points):
    """Callback function for processing user-drawn points."""
    postcode = "EH7 5AF"
    lat,lon = get_lat_lon(postcode=postcode)
    graph = ox.graph_from_point((lat, lon), network_type='walk', dist=5000,dist_type="bbox")
    nodes, edges = ox.graph_to_gdfs(graph)
    bounds = nodes.total_bounds

    road_nodes = get_nearest_road_nodes(graph, points, bounds)

    route = generate_road_route(graph, road_nodes)

    # Compute map center
    map_center = ((bounds[1] + bounds[3]) / 2, (bounds[0] + bounds[2]) / 2)
    plot_route_on_map(graph, route, map_center)


if __name__ == "__main__":
    root = tk.Tk()
    app = DrawApp(root, process_callback=process_drawn_points)
    root.mainloop()
