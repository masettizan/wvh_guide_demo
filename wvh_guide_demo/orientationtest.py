#! /usr/bin/env python3
import rclpy
import time
import csv
import math
import geojson

from rclpy.node import Node
import numpy as np

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
    return graph

def get_orientation_directions(heading, edge, graph):
    # find difference in starting node to end node of edge
    vector_u = np.array([graph[edge[0]]['x'],  graph[edge[0]]['y']]) # vector a - where we are
    vector_v = np.array([graph[edge[1]]['x'],  graph[edge[1]]['y']]) # vector b - where we are going
    
    # heading & head is in vector format
    head, theta, theta_direction = get_angle(heading, vector_u, vector_v)

    if theta == 0.0:
        return head, 'continue'
    
    direction = 'cw' if theta_direction == -1 else 'ccw'
    return head, f'turn {round(theta)} degrees {direction}'
    

def get_angle_direction(heading, goal):
    # these vectors are what were taking the dot product of in get_angle()
    cross = np.cross(heading, goal) 

    if cross > 0:
        return 1 # positive (+), ccw
    elif cross < 0:
        return -1 # negative (-), cw
    else:
        return 0 # colinear (cross == 0), when theta is 180 returns 0 : this is a problem :

def get_angle(heading, a, b):
    # vector difference between 'a' and 'b' - hypotonuse
    goal_vector = b - a
    goal_norm = goal_vector/np.linalg.norm(goal_vector)

    heading_norm = heading/np.linalg.norm(heading)
    cos_theta = np.dot(heading_norm, goal_norm)

    theta = np.arccos(cos_theta)
    theta_direction = get_angle_direction(heading_norm, goal_norm)

    return goal_norm, np.degrees(theta), theta_direction

graph = set_locations()

current, directions = get_orientation_directions(np.array([0, -1]), ('f2_p24', 'f2_p25'), graph)
print(current, directions)