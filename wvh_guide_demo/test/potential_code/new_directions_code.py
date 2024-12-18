#! /usr/bin/env python3

# calculated through our map pgm, if map changes this amount will change
import heapq
import json

import numpy as np


PX_TO_FT = 3/25 
FT_TO_MIN = 4.7/60

'''SETUP'''
def _set_locations():
    with open("/home/masettizan/ros2_ws/src/wvh_guide_demo/svg/WVH.json", "r") as f: #with open("/home/hello-robot/ament_ws/src/wvh_guide_demo/svg/exp/EXP.json", "r") as f: #
        data = json.load(f)
    return data

# calculate cost of edge between a node and it's neighbor
def _calculate_cost(graph, node_id, neighbor_id):
    node = graph[node_id]
    neighbor = graph[neighbor_id]

    if node['type'] == 'elevator' and neighbor['type'] == 'elevator':
        # motivate usage of elevators by making it low cost
        return 0
    
    # distance between coord vectors
    cost = np.asarray((node['x'], node['y'])) - np.asarray((neighbor['x'], neighbor['y']))
    return np.sqrt(cost[0]**2 + cost[1]**2) # magnitude

# find best path from start to end using dijkstra's
def _find_path(graph, start, end):
    queue = []
    total_cost = 0
    
    heapq.heappush(queue, (start, 0))
    path = {start: (None, 0)} # prev node, cost

    while queue:
        node, cost = heapq.heappop(queue)
        
        if node == end:
            result = []
            while node is not None:
                result.append(node)
                node = path[node][0]
            return result[::-1], total_cost # return resvered path

        for neighbor in graph[node]['neighbors']:
            edge_cost = cost + _calculate_cost(graph, node, neighbor)

            if neighbor not in path or edge_cost < path[neighbor][1]:
                path[neighbor] = (node, edge_cost)
                total_cost += edge_cost
                heapq.heappush(queue, (neighbor, edge_cost))

def _find_node_by_type(graph, start_node, type):
    current_floor = graph[start_node]['floor']
    potential = []
    cost = []

    # get potential nodes that could be the goal location
    for key, value in graph.items():
        if value['type'] == type:
            potential.append(key)

            # find the cost of each potential node
            path, weight = _find_path(start_node, type)
            cost.append(weight)
    
    idx = cost.index(min(cost))
    return potential[idx]

def _generate_edges(path):
    edges = []
    for idx in range(len(path) - 1):
        curr_node = path[idx]
        next_node = path[idx + 1]

        edges.append((curr_node, next_node))
    return edges

'''ROTATION'''

# return the amount to turn (in degrees) from the current direction being faced (aka. heading)
def _get_angle(heading, u, v):
    goal_vector = v - u

    goal_norm = goal_vector/np.linalg.norm(goal_vector)
    heading_norm = heading/np.linalg.norm(heading)

    cos_theta = np.dot(heading_norm, goal_norm)
    theta_dir = np.sign(np.cross(heading_norm, goal_norm))

    theta = np.arccos(cos_theta) * theta_dir

    if theta == 0 and np.sum(goal_norm - heading_norm) != 0:
        theta = np.pi

    return np.degrees(theta)

def get_rotation(graph, heading, edge):
    # find difference from start node to end node
    vector_u = np.array([graph[edge[0]]['x'], graph[edge[0]]['y']])
    vector_v = np.array([graph[edge[1]]['x'], graph[edge[1]]['y']])

    theta = _get_angle(heading, vector_u, vector_v) 

    # get new heading with theta in degrees
    curr_heading_rad = np.arctan2(heading[1], heading[0])
    new_heading = (np.degrees(curr_heading_rad) + theta) % 360
    
    return np.array([np.cos(np.radians(new_heading)), np.sin(np.radians(new_heading))]), round(theta)

'''TRANSLATION'''

def get_translation(graph, curr_coords, edge):
    # find difference from start node to end node
    vector_u = np.array([graph[edge[0]]['x'], graph[edge[0]]['y']])
    vector_v = np.array([graph[edge[1]]['x'], graph[edge[1]]['y']])

    if graph[edge[0]]['floor'] != graph[edge[1]]['floor']:
        # take the {type} to the {floor} floor
        return vector_v, (graph[edge[0]]['type'], graph[edge[1]]['floor'])
    
    delta = vector_v - vector_u
    movement = np.sqrt(delta[0]**2 + delta[1]**2)
    return curr_coords + delta, float(round(movement, 2))

'''COMBINE DIRECTIONS'''

def compute_directions(graph, heading, start, end):
    path, cost = _find_path(graph, start, end)
    edges = _generate_edges(path)
    directions = []

    curr_position = np.array([graph[start]['x'], graph[start]['y']])
    for edge in edges:
        heading, theta = get_rotation(graph, heading, edge)
        curr_position, movement = get_translation(graph, curr_position, edge)

        directions.append(('rot', round((-1 * theta/30)%12)))
        if isinstance(movement, tuple):
            directions.append(('vert', f'take the {movement[0]} to floor {movement[1]}'))
        else:
            directions.append(('move', movement))

    return directions, heading, curr_position

'''SIMPLIFICATION'''
def _simplify_rotation(directions):
    idx = 0
    while idx < len(directions):
        type, action = directions[idx]
        if type == 'rot':     
            rot = _clock_to_cardinal(action)
            if not rot or rot == 'forwards':
                directions.pop(idx)
            else:
                directions[idx] = (type, rot)
                idx += 1
        else:
            idx += 1
    return directions


def _combine_translation(directions):
    idx = 0
    while idx < len(directions) - 1:
        type, action = directions[idx]
        next_type, next_action = directions[idx + 1]

        if type == 'move' and next_type == 'move':
            action += next_action
            directions.pop(idx + 1)
            directions[idx] = (type, action)
        else:
            idx += 1
    return directions

def _clock_to_cardinal(action):
    oclock_angles = {
        0: 'forwards',
        1: 'forwards',
        2: 'right',
        3: 'right',
        4: 'right',
        5: 'backwards',
        6: 'backwards',
        7: 'backwards',
        8: 'left',
        9: 'left',
        10: 'left',
        11: 'forwards',
        12: 'forwards'
    }
    return oclock_angles.get(action, None)

def simplify(directions):
    dir = _simplify_rotation(directions)
    dir = _combine_translation(dir)
    result = []

    for step in dir:
        type, action = step
        if type == 'rot':
            if action == 'backwards':
                result.append(f'turn {action}')
            else:
                result.append(f'turn to your {action}')
        elif type == 'move':
            result.append(f'move forward {round(action * PX_TO_FT * FT_TO_MIN, 1)} minutes')
        else:
            result.append(action)
    return result

'''CALLBACK'''

def directions_callback(graph, orientation, curr_pos, goal):
    curr_ori = np.array(orientation, dtype=np.float32)
    if goal in graph.keys():
        goal_node = goal
    else:
        goal_node = _find_node_by_type(graph, curr_pos, goal)
    
    directions, curr_ori, curr_pos = compute_directions(graph, curr_ori, curr_pos, goal_node)
    return ', '.join(simplify(directions)), curr_ori.tolist(), curr_pos.tolist()\
    

graph = {}
    # fill in graph variable
graph = _set_locations()
directions, ori, pos = directions_callback(graph, np.array([0, -1]), 'f1_robot_position', 'f1_p1')
print(directions, ori, pos)
