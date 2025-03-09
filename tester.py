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


    # all_points = np.vstack(filtered_contours)
    # alpha_shape = alphashape(all_points, alpha)
    # if alpha_shape.geom_type == "Polygon":
    #     contour_points = np.array(alpha_shape.exterior.coords)
    #     shape_points = contour_points.tolist()
    #
    # plt.scatter([x[0] for x in shape_points], [x[1] for x in shape_points])
    # plt.show()
    # print(len(shape_points))
    return shape_points

#image_path = r"C:\Users\mgbsa\Desktop\baby-elephant-outline.jpg"
image_path = r"C:\Users\mgbsa\Desktop\boat.png"
extract_shape_outline(image_path)