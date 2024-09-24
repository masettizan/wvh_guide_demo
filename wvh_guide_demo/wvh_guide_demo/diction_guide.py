#! /usr/bin/env python3

import rclpy
import time
import csv

from rclpy.node import Node
import numpy as np

class Graph(Node):
    
    def __init__(self):
        super().__init__('graph')

        self.set_locations()
        self.generate_edges(self.graph)


    # extract information from csv file containing all possible robot goal locations in EXP 120
    def set_locations(self):
        locations_path = '/home/hello-robot/ament_ws/src/wvh_guide_demo/wvh_guide_demo/locations.csv'
        self.graph = {}
        #store locations in dict of tuples --> location : (x, y)
        with open(locations_path, 'r', newline='') as file:
            csv_reader = csv.DictReader(file, delimiter=':')
            for row in csv_reader:
                info = {}
                info['x'] = float(row['position_x'])
                info['y'] = float(row['position_y'])
                info['neighbors'] = row['neighbors'].split(',')
                self.graph[row['location']] = info

    # definition of function 
    def generate_edges(self, path): 
        edges = [] 
    
        for node in path:
            index = path.index(node)
            if node == path[-1]:
                return edges
            
            next_node = path[index + 1]
                
                # if edge exists then append 
            edges.append((node, next_node)) 
        return edges 

    def find_path(self, goal):
        start = 'robot_position'
        explored = []
        queue = [[start]]

        if start == goal:
            return 
        
        while queue:
            #pop first path
            path = queue.pop(0)
            #get last node from path
            node = path[-1]

            if node not in explored:
                neighbors = self.graph[node]['neighbors']
                # go through neighbor nodes
                for neighbor in neighbors:
                    new_path = list(path)
                    new_path.append(neighbor)
                    #push to queue
                    queue.append(new_path)

                    if neighbor == goal:
                        return new_path
                    
                explored.append(node)
        return #error path does not exist

    def get_orientation_directions(self, path):
        current = 180
        directions = []
        edges = self.generate_edges(path)
        for edge in edges:
            delta_x = self.graph[edge[1]]['x'] - self.graph[edge[0]]['x']
            delta_y = self.graph[edge[1]]['y'] - self.graph[edge[0]]['y']

            orientation = (0.0, 0.0) #[x,y]
            #'Yes' if fruit == 'Apple' else 'No'
            if (delta_x != 0.0):
                orientation[0] = np.sign(delta_x)
            elif (delta_y != 0.0):
                orientation[1] = np.sign(delta_y)
            
            
            



            




            directions.append(delta)

    def get_translation_directions(self, path):
        # path to follow
        # only give movement directions
        directions = []
        edges = self.generate_edges(path)
        for edge in edges:
            delta = (self.graph[edge[1]]['x'] - self.graph[edge[0]]['x'], 
                     self.graph[edge[1]]['y'] - self.graph[edge[0]]['y'])
            print(f'move forward {max(delta[0], delta[1])} meters')
            directions.append(delta)
