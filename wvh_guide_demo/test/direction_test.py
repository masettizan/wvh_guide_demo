#! /usr/bin/env python3

import rclpy
import heapq

from rclpy.node import Node
from rclpy.action import ActionServer
import xml.etree.ElementTree as ET
import numpy as np

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import json
import math

CONVERSION = 3/17

#what we return over action server
def directions_callback(graph, elevators, orientation, position, goal):
    current_orientation = np.array(orientation, dtype=np.float32)
    current_position = position
    end_point = goal

    if end_point in graph.keys():
        goal_node = end_point
    else:
        converted_type = get_node_from_type(graph, current_position, end_point)
        # print(f"goal .... {converted_type}")
        goal_node = converted_type

    directions, end_orientation, end_position = get_directions(graph, elevators, current_orientation, current_position, goal_node)
    # print('here', directions)
    ori = end_orientation.astype(np.float32)
    pos = end_position.astype(np.float32)

    return ', '.join(simplify(directions)), ori.tolist(), pos.tolist()

def _set_locations():
    with open("/home/masettizan/ros2_ws/src/wvh_guide_demo/svg/WVH.json", "r") as f: #with open("/home/hello-robot/ament_ws/src/wvh_guide_demo/svg/exp/EXP.json", "r") as f: #
        data = json.load(f)
    return data

def get_node_from_type(graph, start_node, type):
    floor = graph[start_node]['floor']
    options = []
    weights = []

    for key,value in graph.items():
        if value['type'] == '':
            pass
        if value['type'] == type:
            options.append(key)
    
    for key in options:
        path, weight = find_path(start_node, key)
        weights.append(weight)

    idx = weights.index(min(weights))
    return options[idx]

# generate edges for path through graph 
def _generate_edges(path): 
    edges = [] 

    for idx in range(len(path) - 1):
        node = path[idx]
        next_node = path[idx + 1]
        # if edge exists then append 
        edges.append((node, next_node)) 
    return edges 

def _calculate_edge_cost(graph, elevators, node_id, neighbor_id):
    node = graph[node_id]
    neighbor = graph[neighbor_id]

    if node['floor'] != neighbor['floor']:
        if node_id in elevators and neighbor_id in elevators:
            return 0 # motivate usage of elevators by making it low cost
        else: # only other option is stairs, seems reduntant to do elif
            return 10 # avg stair case for 10ft ceilings is 3m, assuming exp is 30ft ceilings

    node_coord = np.asarray((node['x'], node['y'])) 
    neighbor_coord = np.asarray((neighbor['x'], neighbor['y']))

    cost = node_coord - neighbor_coord # distance between coord vectors
    return np.sqrt(cost[0]**2 + cost[1]**2) # magnitude of cost vector

# find path through dijkstra from start to goal in the graph
def find_path(graph, elevators, start, goal):
    queue = []
    total_weight = 0

    heapq.heappush(queue, (0, start))
    path = {start: (None, 0)} # prev node, cost

    while queue:
        cost, node = heapq.heappop(queue)
        
        if node == goal:
            result = []
            while node is not None:
                result.append(node)
                node = path[node][0]
            return result[::-1], total_weight # return resvered path

        for neighbor in graph[node]['neighbors']:
            weight = cost + _calculate_edge_cost(graph, elevators, node, neighbor)

            if neighbor not in path or weight < path[neighbor][1]:
                path[neighbor] = (node, weight)
                total_weight += weight
                heapq.heappush(queue, (weight, neighbor))

def _get_orientation_directions(graph, heading, edge):
    # find difference in starting node to end node of edge
    vector_u = np.array([graph[edge[0]]['x'], graph[edge[0]]['y']]) # vector a - where we are
    vector_v = np.array([graph[edge[1]]['x'], graph[edge[1]]['y']]) # vector b - where we are going
    
    # heading & head is in vector format
    head, theta, theta_direction = _get_angle(heading, vector_u, vector_v)
    print(heading, head, theta)
    #manipulate heading to turn the amt of head
    if theta == 0.0:
        return head, 0
    
    return _get_new_facing(heading, np.radians(theta)), (round(theta), theta_direction)        

def _get_new_facing(orientation, turn_amt):
    rot = np.array([[np.cos(turn_amt), -np.sin(turn_amt)],
                    [np.sin(turn_amt), np.cos(turn_amt)]])
    print('new:',rot @ orientation)
    return rot @ orientation

def _get_angle_direction(heading, goal):
    # these vectors are what were taking the dot product of in get_angle()
    cross = np.cross(heading, goal) 

    if cross >= 0:
        return 1 # positive (+), ccw
    else:
        return -1 # negative (-), cw

def _get_angle(heading, u, v):
    # vector difference between 'a' and 'b' - hypotonuse
    goal_vector = v - u

    if not np.any(goal_vector): #elevator
        return -1*heading, 180, 1

    goal_norm = goal_vector/np.linalg.norm(goal_vector)

    heading = np.array([0,1])
    heading_norm = heading/np.linalg.norm(heading)
    cos_theta = np.dot(heading_norm, goal_norm)

    theta = np.arccos(cos_theta)
    theta_direction = _get_angle_direction(heading_norm, goal_norm)

    return goal_norm, np.degrees(theta), theta_direction

# return directions for movement for given edge, update current position
def _get_translation_directions(graph, elevators, current, edge): 
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
    current = current + delta 

    move = np.sqrt(delta[0]**2 + delta[1]**2)
    direction = math.ceil(float(round(move, 2)))
    return current, direction

def get_directions(graph, elevators, heading, start, goal):
    path, weight = find_path(graph, elevators, start, goal)
    edges = _generate_edges(path)

    directions = []
    orientation = heading
    position = np.array([graph[start]['x'], graph[start]['y']])

    # for each edge in the path calculate directions
    for edge in edges:
        print('orienttion1', orientation)
        orientation, turn = _get_orientation_directions(graph, orientation, edge)
        position, movement = _get_translation_directions(graph, elevators, position, edge)
        print('facing', heading)
        print('orienttion2', orientation)
        print('turn to', turn)
        directions.append(('rot', turn))
        directions.append(('move', movement))
        heading = orientation
        
        if isinstance(movement, tuple) and 'elevator' in movement[0]: # assuming all elevators face the same direction: -1, 0 aka. pi
            orientation = np.array([-1, 0]) 
            directions.append(('vert', 'turn to face exit of elevator'))
        elif isinstance(movement, tuple) and 'stairs' in movement[0]: # assuming all stair exits face the same direction: -1, 0 aka. pi
            orientation = np.array([-1, 0]) 
            directions.append(('vert', 'continue facing exit of stairs'))


    # return directions in array, along with updated user info
    return directions, orientation, position

def _simplify_rotation(directions):
    # print(directions)
    new_directions = []
    for step in directions:
        type, action = step
        # print('rot', action)
        if type == 'rot':
            if isinstance(action, tuple):
                # i dont need to know where i am because where your facing is always 12 o'clock
                theta, sign = action
            # print('simp', theta, sign)
                # if round(-1*sign*theta/30)%12 == 0:
                #     continue
                new_directions.append((type, round(-1*sign*theta/30)%12)) # 360/12 = 30
            # action is not a touple and is therefore 0 and therefore not added
            else:
                #continue
                new_directions.append((type, 0))
        else:
            new_directions.append(step)
    return new_directions

def _simplify_translation(directions):
    index = 0

    while index < len(directions) - 1:
        type, action = directions[index]
        next_type, next_action = directions[index + 1]

        if type == 'move' and next_type == 'move':
            if isinstance(action, int) and isinstance(next_action, int):
                action = action + next_action
                directions.pop(index + 1)
                directions[index] = (type, action)
            else:
                index += 1
        else:
            index += 1
    return directions


def clock_to_cardinal(action):
    # The 12 o'clock position is straight ahead (0 degrees)
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
    # print(directions)
    dir = _simplify_rotation(directions)
    dir = _simplify_translation(dir)
    # print(dir)

    result = []

    for step in dir:
        type, action = step
        if type == 'rot':
            rot = clock_to_cardinal(action)
            if rot:
                result.append(f"turn to your {action}")
        elif type == 'move':
            if isinstance(action, tuple):
                result.append(f'take the {action[0]} to floor {action[1]}')
            else:
                distance = round(action * 4.7 / 60)
                result.append(f'move forward {distance} minutes')
        else:
            result.append(action)

    return result

graph = {}
    # fill in graph variable
graph = _set_locations()
elevators = ['f1_elevator', 'f2_elevator', 'f3_elevator', 'f4_elevator']
directions, ori, pos = directions_callback(graph, elevators, np.array([0, -1]), 'f1_robot_position', 'f1_p9')
print(directions, ori, pos)