#! /usr/bin/env python3

# # import dictionary for graph 
from collections import defaultdict 
import csv
import math
from math import atan2
from sensor_msgs.msg import JointState
import numpy as np
# function for adding edge to graph 
# graph = defaultdict(list) 
# graph["u"].append('v')
# graph['t'] = 'e'
# print(graph)

def set_locations():
    locations_path = '/home/hello-robot/ament_ws/src/wvh_guide_demo/wvh_guide_demo/locations.csv'
    graph = {}
    #store locations in dict of tuples --> location : (x, y)
    with open(locations_path, 'r', newline='') as file:
        csv_reader = csv.DictReader(file, delimiter=':')
        for row in csv_reader:
            info = {}
            info['x'] = float(row['position_x'])
            info['y'] = float(row['position_y'])
            info['neighbors'] = row['neighbors'].split(',')
            graph[row['location']] = info

    return graph

def generate_edges(path): 
    edges = [] 
    print(len(path))

    for i in range(len(path) - 1):
        print(path)
        node = path[i]
        next_node = path[i + 1]
        print(node,next_node)
            
            # if edge exists then append 
        edges.append((node, next_node)) 
    print (edges)
    return edges 

def get_translation_directions( edge, graph):
    # path to follow
    # only give movement directions
    print(graph[edge[1]], graph[edge[0]])
    delta = ( graph[edge[1]]['x'] -  graph[edge[0]]['x'], 
                graph[edge[1]]['y'] -  graph[edge[0]]['y'])
    
    # move = delta[0] if max(abs(delta[0]), abs(delta[1])) == abs(delta[0]) else delta[1]
    #always pos cause neg is happening in turning
    val = (f'move forward {float(round(max(abs(delta[0]), abs(delta[1]))))} meters')
    return val

 #cardinal and constant 

def get_orientation(current, delta_x, delta_y):
    # print(delta_x)
    # print(delta_y)
    # print(current)
    print(current - delta_x*math.pi/2)
    # print((current - delta_x*math.pi/2 - delta_y*math.pi/2)%(2*math.pi))
    return (current - delta_x*math.pi/2 - delta_y*math.pi/2)%(2*math.pi)

def get_goal(delta_x, delta_y):
    x = np.sign(delta_x)
    y = np.sign(delta_y)

    if x > 0.0:
        return 0.0
    elif x < 0.0:
        return math.pi
    elif y > 0.0:
        return math.pi/2
    elif y < 0.0:
        return 3*math.pi/2
    else:
        None
        # its the same point

#cardinal and constant
def get_orientation_directions(current, edge, graph):
    current = current # 0/2pi , pi/2, pi, 3pi/2
    
    print("curr:", current)
    delta_x =  graph[edge[1]]['x'] - graph[edge[0]]['x']
    delta_y =  graph[edge[1]]['y'] - graph[edge[0]]['y']
    goal = []
    goal_orientaion = get_goal(delta_x, delta_y)
        

    directions = [] #[x,y]
    #'Yes' if fruit == 'Apple' else 'No'
    if goal_orientaion == current:
        print("continue facing your direction")
        return
    while current != goal_orientaion:
        print(current,",",goal_orientaion)
        if delta_x == 0.0 and delta_y == 0.0:
            print("done")
            return
        x = np.sign(delta_x)
        y = np.sign(delta_y) 

        if current == math.pi or current == math.pi/2:
            orientation = get_orientation(current, x, y)
            if (x < 0.0 and y == 0.0) or (x == 0.0 and y < 0.0):
                print('left')
                directions.append('left')
            else:
                print('right')
                directions.append('right')
        elif current == 3*math.pi/2 or current == 2*math.pi or current == 0.0:
            orientation = get_orientation(current, -1*x, -1*y)
            if (x < 0.0 and y == 0.0) or (x == 0.0 and y < 0.0):
                print('right')
                directions.append('right')
            else:
                print('left')
                directions.append('left')



        # print(orientation)

        current = orientation
        #fix pls plsa
    # directions.append(orientation)
    print("lets see") 
    return directions, current

# edges = generate_edges(['robot_position','drake_desk','demo_table'])
graph = set_locations()
edges = generate_edges([ 'drake_desk', 'emily_desk' , 'drake_desk'])


print(dir)
directions = []
curr = math.pi
for edge in edges:
    ori, curr = get_orientation_directions(curr, edge, graph)
    move = get_translation_directions(edge,graph)
    for turn in ori:
        directions.append(turn)
    directions.append(move)
print(directions)