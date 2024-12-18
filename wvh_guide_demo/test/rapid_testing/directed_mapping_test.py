#! /usr/bin/env python3
import xml.etree.ElementTree as ET

import geojson
import matplotlib.pyplot as plt 

import geopandas as gpd
import networkx as nx
import matplotlib.pyplot as plt
from shapely.geometry import Point
import matplotlib.image as mpimg
from PIL import Image


def svg():
    tree = ET.parse('/home/masettizan/ros2_ws/src/wvh_guide_demo/svg/WVH.svg')
    root = tree.getroot()
    info = {}
    
    def element_to_dict(element, info):
        if element.attrib.get('id') is not None:
            element_dict = {}
            element_dict['x'] = float(element.attrib.get('x'))
            element_dict['y'] = float(element.attrib.get('y'))
            element_dict['neighbors'] = element.attrib.get('neighbors').split(',')
            element_dict['floor'] = int(element.attrib.get('floor'))
            
        
            info[element.attrib.get('id')] = element_dict
        return info
    
    if list(root):
        for child in list(root):
            info = element_to_dict(child, info)

    return info

def add_node_to_graph(graph, floor, g,size):
    nodes = []
    for id, data in graph.items():
        if data['floor'] == floor:
            print(size)
            g.add_node(id, pos=(data['x'] * size[0] - size[1], data['y']*size[2] +size[3]), floor=data['floor'], neighbors=data['neighbors'])
            nodes.append(id)
    return nodes

def add_edges_to_graph(graph, floor,g):
    edges = []
    for id, data in graph.items():
        if data['floor'] == floor:
            for neighbor in data['neighbors']:
                if id[:2] == neighbor[:2]:
                    g.add_edge(id, neighbor)
                    if (neighbor, id) not in edges:
                        edges.append((id, neighbor))
    return edges

def setup_svg(graph):
    plt.figure(figsize=(10,8))

    img=mpimg.imread('/home/masettizan/ros2_ws/src/wvh_guide_demo/svg/WVH_1_size.jpg')
    plt.imshow(img)
    draw_floor(graph, 1, 'purple')
    draw_floor(graph, 2, 'orange')
    draw_floor(graph, 3, 'red')
    draw_floor(graph, 4, 'lightblue')
    plt.show()

def setup_map(graph):

    graphs = [G1,G2,G3,G4]
    colors = ['purple', 'orange', 'red', 'lightblue']
    
    plt.ion()
    fig, axes = plt.subplots(nrows=2, ncols=2, figsize = [10, 8])
    for i in range(4):
        image = mpimg.imread(f'/home/masettizan/ros2_ws/src/wvh_guide_demo/svg/images/WVH_{i+1}.jpg')
        G, ax = edge_location(i+1, axes )
        ax.imshow(image)
        
        draw_floor(graph, i+1 , colors[i],G, sizing[i],ax)

    plt.show(block=False)
    edges = [('f1_p1', 'f1_elevator'), ('f1_elevator', 'f2_elevator'), ('f2_elevator', 'f2_p0'), ('f2_p0', 'f2_p32'), ('f2_p32', 'f2_p31'), ('f2_p31', 'f2_p29'), ('f2_p29', 'f2_p28'), ('f2_p28', 'f2_p27'), ('f2_p27', 'f2_240')]
    for edge in edges:
        draw_edge([edge], axes)
    plt.show()

def draw_floor(graph, floor, color,g,size, ax):
    nodes = add_node_to_graph(graph, floor,g,size)
    edges = add_edges_to_graph(graph, floor,g)
    pos = nx.get_node_attributes(g, 'pos')
    nx.draw(g, 
            pos, 
            nodelist=nodes, 
            edgelist=edges, 
            node_size=10, 
            node_color=color, 
            edge_color=color, 
            style='dashed', 
            with_labels=False, 
            ax=ax)

def draw_edge(edge, ax):
    u = edge[0][0]
    v = edge[0][1]
    if u[:2] == v[:2]:
        G, axe = edge_location(int(v[1:2]),ax)
        pos = nx.get_node_attributes(G, 'pos')
        nx.draw_networkx_edges(G, pos, edgelist=edge, width=2, ax=axe)
    plt.pause(1)

def edge_location(floor, ax):
    if floor == 1:
        G=G1
        axe=ax[0,0]
    elif floor == 2:
        G=G2
        axe=ax[0,1]
    elif floor == 3:
        G=G3
        axe=ax[1,0]
    else:
        G=G4
        axe=ax[1,1]
    return G, axe

G1 = nx.Graph()
G2 = nx.Graph()
G3 = nx.Graph()
G4 = nx.Graph()
graph = svg()

sizing = [[2.63, 840, 2.63, 55],
              [1.05, 320, 1.05, 0],
              [.89, 240, .89, 30],
              [1.8, 545, 1.8, -820]]
setup_map(graph)

# draw_path(('f1_p1', 'f1_p2'))
input("press enter")
plt.close()
# plt.show()