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
    locations = {}
    #store locations in dict of tuples --> location : (x, y)
    with open(locations_path, 'r', newline='') as file:
        csv_reader = csv.DictReader(file, delimiter=':')
        for row in csv_reader:
            info = {}
            info['x'] = float(row['position_x'])
            info['y'] = float(row['position_y'])
            info['neighbors'] = row['neighbors'].split(',')
            locations[row['location']] = info

    return locations



# def find_path(graph,goal):
#     start = 'robot_position'
#     explored = []
#     queue = [[start]]

#     if start == goal:
#         return 
    
#     while queue:
#         #pop first path
#         path = queue.pop(0)
#         #get last node from path
#         node = path[-1]

#         if node not in explored:
#             neighbors = graph[node]['neighbors']
#             # go through neighbor nodes
#             for neighbor in neighbors:
#                 new_path = list(path)
#                 new_path.append(neighbor)
#                 #push to queue
#                 queue.append(new_path)

#                 if neighbor == goal:
#                     return new_path
                
#             explored.append(node)
#     return #error path does not exist

# def get_directions(path, graph):
#     # path to follow
#     # assume facing robot and will have to turn 90 degrees to your right
#     # assume every node is conencted by 90 degree angles so order of motion doesn't matter
#     directions = []
#     for node in path:
#         index = path.index(node)
#         if node == path[-1]:
#             return directions
        
#         next_node = path[index + 1]
#         delta = (float(graph[next_node]['x']) - float(graph[node]['x']), 
#                    float( graph[next_node]['y']) - float(graph[node]['y']))
#         directions.append(delta)



# l = set_locations()
# path = find_path(l, 'emily_desk')
# print(get_directions(path,l))


# sample graph implemented as a dictionary
# graph = {'A': ['B', 'C', 'E'],
#          'B': ['A','D', 'E'],
#          'C': ['A', 'F', 'G'],
#          'D': ['B'],
#          'E': ['A', 'B','D'],
#          'F': ['C'],
#          'G': ['C']}


# camera_pan = -1 * atan2(0.0, 1.7)
# print(math.degrees(camera_pan))

# camera_pan = -1 * atan2(-1.7, 0.0)
# print(math.degrees(camera_pan))

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
def get_translation_directions( path, graph):
        # path to follow
        # only give movement directions
        directions = []
        edges =  generate_edges(path)
        for edge in edges:
            print(graph[edge[1]], graph[edge[0]])
            delta = ( graph[edge[1]]['x'] -  graph[edge[0]]['x'], 
                      graph[edge[1]]['y'] -  graph[edge[0]]['y'])
            print(f'move forward {max(delta[0], delta[1])} meters')
            directions.append(delta)
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
def get_orientation_directions(path, graph):
    current = math.pi # 0/2pi , pi/2, pi, 3pi/2
    directions = []
    edges = generate_edges(path)
    for edge in edges:
        print("curr:", current)
        delta_x =  graph[edge[1]]['x'] - graph[edge[0]]['x']
        delta_y =  graph[edge[1]]['y'] - graph[edge[0]]['y']
        goal = []
        goal_orientaion = get_goal(delta_x, delta_y)
         

        direction = 0.0 #[x,y]
        #'Yes' if fruit == 'Apple' else 'No'
        if goal_orientaion == current:
            print("continue facing your direction")
            continue
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
            
        # directions.append(orientation)
        print("lets see")
    return directions


# edges = generate_edges(['robot_position','drake_desk','demo_table'])
graph = set_locations()

dir = get_orientation_directions([ 'drake_desk', 'emily_desk' , 'drake_desk'], graph)
print(dir)