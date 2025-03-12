import tkinter as tk
import osmnx as ox
import networkx as nx
import folium
import numpy as np



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

def get_nearest_road_nodes(graph, shape_points, map_bounds):
    lat_min, lat_max, lon_min, lon_max = map_bounds
    points_array = np.array(shape_points)
    print(points_array)
    points_array[:, 0] = lat_min + (points_array[:, 0] / 500) * (lat_max - lat_min)
    points_array[:, 1] = lon_min + (points_array[:, 1] / 500) * (lon_max - lon_min)
    print(points_array)
    road_nodes = []
    for lat, lon in points_array:
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


def plot_route_on_map(graph, route_nodes, center_latlon, output_file="route.html"):
    m = folium.Map(location=center_latlon, zoom_start=14)
    route_coords = [(graph.nodes[n]['x'], graph.nodes[n]['y']) for n in route_nodes]
    folium.PolyLine(route_coords, color='blue', weight=5, opacity=0.8).add_to(m)
    m.save(output_file)
    print(f"✅ Route saved as {output_file}")





if __name__ == "__main__":
    root = tk.Tk()
    app = DrawApp(root)
    root.mainloop()
    coordinates = app.points
    place_name = "Haywards Heath, UK"
    graph = ox.graph_from_place(place_name, network_type="walk")
    bounds = ox.geocode_to_gdf(place_name).total_bounds
    print(bounds)
    road_nodes = get_nearest_road_nodes(graph, coordinates, bounds)
    print(road_nodes)
    route = generate_road_route(graph, road_nodes)

    map_center = ((bounds[1] + bounds[3]) / 2, (bounds[0] + bounds[2]) / 2)
    plot_route_on_map(graph, route, map_center)