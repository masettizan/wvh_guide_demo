#! /usr/bin/env python3

import geojson
import matplotlib.pyplot as plt 

import geopandas as gpd
import networkx as nx
import matplotlib.pyplot as plt
from shapely.geometry import Point

def set_locations():
    graph = {}
    with open('/home/masettizan/ros2_ws/src/wvh_guide_demo/json/floors.geojson') as f:
        data = geojson.load(f)
        
    for feature in data['features']:
        info = {}
        info['x'] = float(feature['geometry']['coordinates'][0])
        info['y'] = float(feature['geometry']['coordinates'][1])
        info['neighbors'] = feature['properties']['neighbors']
        info['floor'] = feature['properties']['floor']
        graph[feature['properties']['id']] = info
    
    elevators = ['f1_p7', 'f2_p1', 'f3_p1']

    return graph, elevators

def add_node_to_graph(graph, floor):
    nodes = []
    for id, data in graph.items():
        if data['floor'] == floor:
            G.add_node(id, pos=(data['x'], data['y']), floor=data['floor'], neighbors=data['neighbors'])
            nodes.append(id)
    return nodes

def add_edges_to_graph(graph, floor):
    edges = []
    for id, data in graph.items():
        if data['floor'] == floor:
            for neighbor in data['neighbors']:
                if id[:2] == neighbor[:2]:
                    G.add_edge(id, neighbor)
                    if (neighbor, id) not in edges:
                        edges.append((id, neighbor))
    return edges

def setup_map(graph):
    plt.ion()
    plt.figure(figsize=(10,8))
    draw_floor(graph, 1, 'purple')
    draw_floor(graph, 2, 'orange')
    draw_floor(graph, 3, 'red')
    # plt.show()

def draw_floor(graph, floor, color):
    nodes = add_node_to_graph(graph, floor)
    edges = add_edges_to_graph(graph, floor)
    pos = nx.get_node_attributes(G, 'pos')
    for i in range(100):
        print("hiu")
    nx.draw(G, pos, nodelist=nodes, edgelist=edges, node_size=20, node_color=color, edge_color=color, style='dashed', with_labels=False)

def draw_path(path):
    pos = nx.get_node_attributes(G, 'pos')
    nx.draw_networkx_edges(G, pos, edgelist=path, width=2)

G = nx.Graph()
graph, elevators = set_locations()
setup_map(graph)
draw_path(('f1_p1', 'f1_p2'))
input("press enter")
plt.close()
# plt.show()