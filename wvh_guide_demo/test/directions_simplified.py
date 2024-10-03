#! /usr/bin/env python3

import rclpy
import time
import csv
import math
import geojson
import heapq

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

def get_orientation_directions_og(graph, heading, edge):
    # find difference in starting node to end node of edge
    vector_u = np.array([graph[edge[0]]['x'], graph[edge[0]]['y']]) # vector a - where we are
    vector_v = np.array([graph[edge[1]]['x'], graph[edge[1]]['y']]) # vector b - where we are going
    
    # heading & head is in vector format
    head, theta, theta_direction = get_angle(heading, vector_u, vector_v)

    if theta == 0.0:
        return head, 'continue'
    
    direction = 'cw' if theta_direction == -1 else 'ccw'
    return head, f'turn {round(theta)} degrees {direction}'      

def get_translation_directions_og(graph, current, edge, elevators):
    vector_u = np.array([graph[edge[0]]['x'], graph[edge[0]]['y']]) # vector a - where we are
    vector_v = np.array([graph[edge[1]]['x'], graph[edge[1]]['y']]) # vector b - where we are going

    if graph[edge[0]]['floor'] != graph[edge[1]]['floor']:
        if edge[0] in elevators and edge[1] in elevators:
            transport = 'elevator' 
        else:
            transport = 'stairs'
        direction = f'take {transport} to floor {graph[edge[1]]["floor"]}'

        return vector_v, direction
    delta = vector_v - vector_u
    current = current + delta #(current[0] + delta[0], current[1] + delta[1])

    move = np.sqrt(delta[0]**2 + delta[1]**2) #max(abs(delta[0]), abs(delta[1]))
    direction = f'move forward {float(round(move, 2))} meters'
    return current, direction

def get_angle_direction(heading, goal):
    # these vectors are what were taking the dot product of in get_angle()
    cross = np.cross(heading, goal) 

    if cross >= 0:
        return 1 # positive (+), ccw
    else: # elif cross < 0:
        return -1 # negative (-), cw
    # else:
    #     return 0 # colinear (cross == 0), when theta is 180 returns 0 : this is a problem :

def get_angle(heading, u, v):
    # vector difference between 'a' and 'b' - hypotonuse
    goal_vector = v - u
    goal_norm = goal_vector/np.linalg.norm(goal_vector)

    heading_norm = heading/np.linalg.norm(heading)
    cos_theta = np.dot(heading_norm, goal_norm)

    theta = np.arccos(cos_theta)
    theta_direction = get_angle_direction(heading_norm, goal_norm)

    return goal_norm, np.degrees(theta), theta_direction

def get_translation_directions(graph, current, edge, elevators):
    vector_u = np.array([graph[edge[0]]['x'], graph[edge[0]]['y']]) # vector a - where we are
    vector_v = np.array([graph[edge[1]]['x'], graph[edge[1]]['y']]) # vector b - where we are going

    if graph[edge[0]]['floor'] != graph[edge[1]]['floor']:
        if edge[0] in elevators and edge[1] in elevators:
            transport = 'elevator' 
        else:
            transport = 'stairs'
        direction = (transport, graph[edge[1]]["floor"])

        return vector_v, direction
    delta = vector_v - vector_u
    current = current + delta #(current[0] + delta[0], current[1] + delta[1])

    move = np.sqrt(delta[0]**2 + delta[1]**2) #max(abs(delta[0]), abs(delta[1]))
    direction = float(round(move, 2))
    return current, direction

def get_orientation_directions(graph, heading, edge):
    # find difference in starting node to end node of edge
    vector_u = np.array([graph[edge[0]]['x'], graph[edge[0]]['y']]) # vector a - where we are
    vector_v = np.array([graph[edge[1]]['x'], graph[edge[1]]['y']]) # vector b - where we are going
    
    # heading & head is in vector format
    head, theta, theta_direction = get_angle(heading, vector_u, vector_v)
    
    if theta == 0.0:
        return head, 0
    
    # direction = 'cw' if theta_direction == -1 else 'ccw'
    return head, (round(theta), theta_direction)      

# return directions from start position to goal position on graph using BFS
# update current orientation and current position
def get_directions(graph, heading, start, goal):
    elevators = ['f1_p7', 'f2_p1', 'f3_p1']
    path = ['f1_p1', 'f1_p2', 'f1_p3', 'f1_p4', 'f1_p10', 'f1_p11', 'f1_p13', 'f1_p5', 'f1_p6', 'f1_p7', 'f3_p1', 'f3_p2']
    edges = [('f1_p1', 'f1_p2'), ('f1_p2', 'f1_p3'), ('f1_p3', 'f1_p4'), ('f1_p4', 'f1_p10'), ('f1_p10', 'f1_p11'), ('f1_p11', 'f1_p13'), ('f1_p13', 'f1_p5'), ('f1_p5', 'f1_p6'), ('f1_p6', 'f1_p7'), ('f1_p7', 'f3_p1'), ('f3_p1', 'f3_p2')]

    directions = []
    # current position and orientation in room/graph
    orientation = heading
    position = np.array([graph[start]['x'], graph[start]['y']])

    # for each edge in the path calculate directions
    for edge in edges:
        orientation, turn = get_orientation_directions(graph, orientation, edge)
        position, movement = get_translation_directions(graph, position, edge, elevators)
        # some positions may require more than one turn at a time (aka U-turns, etc.)
        
        directions.append(('rot', turn))
        directions.append(('move', movement))
        
        if isinstance(movement, tuple) and 'elevator' in movement[0]: # assuming all elevators face the same direction: -1, 0 aka. pi
            orientation = np.array([-1, 0]) 
            directions.append(('vert', 'turn to face exit of elevator'))
        elif isinstance(movement, tuple) and 'stairs' in movement[0]: # assuming all stair exits face the same direction: -1, 0 aka. pi
            orientation = np.array([-1, 0]) 
            directions.append(('vert', 'continue facing exit of stairs'))

    # return directions in array, along with updated user info
    return directions, orientation, position

def simplify_rotation(directions):
    new_directions = []
    for step in directions:
        type, action = step
        if type == 'rot':
            if isinstance(action, tuple):
                # i dont need to know where i am because where your facing is always 12 o'clock
                
                theta, sign = action
                new_directions.append((type, round(-1*sign*theta/30)%12)) # 360/12 = 30
            # action is not a touple and is therefore 0 and therefore not added
            else:
                continue
        else:
            new_directions.append(step)
    return new_directions

def simplify_translation(directions):
    index = 0

    while index < len(directions) - 1:
        type, action = directions[index]
        next_type, next_action = directions[index + 1]

        if type == 'move' and next_type == 'move':
            if isinstance(action, float) and isinstance(next_action, float):
                action = action + next_action
                directions.pop(index + 1)
                directions[index] = (type, action)
            else:
                index += 1
        else:
            index += 1
    return directions

def simplify(directions):
    dir = simplify_rotation(directions)
    dir = simplify_translation(dir)

    result = []

    for step in dir:
        type, action = step
        if type == 'rot':
            result.append(f"turn to your {action} o'clock")
        elif type == 'move':
            if isinstance(action, tuple):
                result.append(f'take the {action[0]} to floor {action[1]}')
            else:
                result.append(f'move forward {action} meters')
        else:
            result.append(action)

    return result

graph = set_locations()
dir, ori, pos = get_directions(graph, np.array([0, 1]), 'f1_p7', 'f3_p2')
print(simplify(dir))