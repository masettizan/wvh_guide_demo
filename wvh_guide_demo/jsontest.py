#! /usr/bin/env python3

import geojson
import matplotlib.pyplot as plt 
from descartes import PolygonPatch

import geopandas as gpd
import networkx as nx
import matplotlib.pyplot as plt
from shapely.geometry import Point

with open("/home/masettizan/ros2_ws/src/wvh_guide_demo/json/floors.geojson") as json_file:
    json_data = geojson.load(json_file) # or geojson.load(json_file)
print(json_data.keys())
print(type(json_data['features']))

poly = json_data['features'][0]
print (poly)
print(poly["geometry"])
print(poly.geometry.coordinates)

def layered_gpd():
    #geopandas attempot layered
    floor1 = gpd.read_file("/home/masettizan/ros2_ws/src/wvh_guide_demo/json/floor1.geojson")
    floor2 = gpd.read_file("/home/masettizan/ros2_ws/src/wvh_guide_demo/json/floor2.geojson")

    fig, ax = plt.subplots()

    floor1.plot(ax=ax, color='white', edgecolor='black')


    floor2.plot(ax=ax, marker='o', color='red', markersize=5)


    plt.show()

geo_data = json_data['features']
# Load GeoJSON file using geopandas
# having ogr error w this - solution create own json interpreter
    # geo_data = gpd.read_file("/home/masettizan/ros2_ws/src/wvh_guide_demo/json/floors.geojson")
    # print(geo_data)
# Initialize a graph
G = nx.Graph()

# Function to add nodes and edges from geometries
def add_geo_to_graph_gds(geo_data):
    for i, row in geo_data.iterrows():
        # neighbors = ",".join(row.neighbors)
        geometry = row.geometry
        if isinstance(geometry, Point):  # If geometry is a Point, add as a node
            G.add_node(i, pos=(geometry.x, geometry.y), floor=row.floor, id=row.id)

'''
{"geometry": {
    "coordinates": [0.0, 0.0],
    "type": "Point"
    }, 
"properties": {
    "floor": 1, 
    "id": "f1_p1", 
    "neighbors": ["f1_p2"]
    }, 
"type": "Feature"
}
'''            
def add_geo_to_graph(geo_data):
    color = []
    edge = []
    for i in range(len(geo_data)):
        node = geo_data[i]
        geometry = node.geometry
        coords = (geometry.coordinates[0], geometry.coordinates[1])
        properties = node.properties
        # print(properties)

        G.add_node(node_for_adding=properties["id"], pos=coords, floor=properties['floor'], neighbors=properties['neighbors'])
        if properties['floor'] == 1:
            color.append('red')
        elif properties['floor'] == 2:
            color.append('blue')
        else:
            color.append('green')

    for node, data in G.nodes.items():
        for neighbor in data["neighbors"]:
            G.add_edge(node, neighbor)
            if node[:2] != neighbor[:2]:
                print(node, neighbor)
                edge.append((node, neighbor))

    return color, edge

        

# Add nodes and edges from the GeoJSON geometries to the graph

color,edge = add_geo_to_graph(json_data['features'])

# Get positions for nodes
pos = nx.get_node_attributes(G, 'pos')

# Draw the graph
plt.figure(figsize=(10, 8))
nx.draw(G, pos, node_size=30, node_color=color, style="dashed",with_labels=False)
nx.draw_networkx_edges(G, pos, edgelist=edge, edge_color="purple")

# Plot the geometries as well for comparison
# geo_data.plot(ax=plt.gca(), alpha=0.5, edgecolor='black')

# Show the plot
plt.show()
