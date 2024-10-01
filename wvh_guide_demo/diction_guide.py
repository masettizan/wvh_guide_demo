#! /usr/bin/env python3

import rclpy
import time
import csv
import math
import geojson

from rclpy.node import Node
import numpy as np

class Graph(Node):
    
    def __init__(self):
        super().__init__('graph')
        self.graph = {}
        # fill in graph variable
        self.set_locations()


    # extract information from csv file containing all possible robot goal locations in EXP 120
    def set_locations_csv(self):
        locations_path = '/home/hello-robot/ament_ws/src/wvh_guide_demo/wvh_guide_demo/locations.csv'
        #store locations in dict of tuples --> location : (x, y)
        with open(locations_path, 'r', newline='') as file:
            csv_reader = csv.DictReader(file, delimiter=':')
            for row in csv_reader:
                info = {}
                info['x'] = float(row['position_x'])
                info['y'] = float(row['position_y'])
                info['neighbors'] = row['neighbors'].split(',')
                self.graph[row['location']] = info
    
    def set_locations(self):
        with open('/home/masettizan/ros2_ws/src/wvh_guide_demo/json/floors.geojson') as f:
            data = geojson.load(f)
            
        for feature in data['features']:
            info = {}
            info['x'] = float(feature['geometry']['coordinates'][0])
            info['y'] = float(feature['geometry']['coordinates'][1])
            info['neighbors'] = feature['properties']['neighbors']
            info['floor'] = feature['properties']['floor']
            self.graph[feature['properties']['id']] = info

    # generate edges for path through graph 
    def generate_edges(self, path): 
        edges = [] 
    
        for idx in range(len(path) - 1):
            node = path[idx]
            next_node = path[idx + 1]
            # if edge exists then append 
            edges.append((node, next_node)) 
        return edges 

    # find path through BFS from start to goal in the graph
    def find_path(self, start, goal):
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

    # return orientation for given difference in x and y from edges
    def get_goal_orientation(self, x_diff, y_diff):
        x_diff = np.sign(x_diff)
        y_diff = np.sign(y_diff)

        if x_diff > 0.0:
            return 0.0
        elif x_diff < 0.0:
            return math.pi
        elif y_diff > 0.0:
            return math.pi/2
        elif y_diff < 0.0:
            return 3*math.pi/2
        else:
            # its the same point
            return None

    # return new orientation in terms of the unit circle in radians      
    def get_new_orientation(self, current, x, y):
        return (current - x*math.pi/2 - y*math.pi/2)%(2*math.pi)
    
    # return directions for turning for given edge and starting orientation
    def get_orientation_directions(self, start_orientation, edge):
        current = start_orientation
        # find difference in starting node to end node of edge
        delta_x =  self.graph[edge[1]]['x'] - self.graph[edge[0]]['x']
        delta_y =  self.graph[edge[1]]['y'] - self.graph[edge[0]]['y']
        # find what direction we need to face to be able to travel straight forward to it
        goal_orientation = self.get_goal_orientation(delta_x, delta_y)  
        

        directions = []
        # if already facing direction continue
        if goal_orientation == current or None:
            return current, ['continue facing current direction']
        # keep turning until goal orientation is acheived
        while current != goal_orientation:
            print(goal_orientation, current)
            # if edge have same location for the start and end node 
            if delta_x == 0.0 and delta_y == 0.0:
                return current, ['arrived']
            # get sign of difference in position
            x = np.sign(delta_x)
            y = np.sign(delta_y)

            # if facing pi or pi/2, aka. 180 or 90 degrees
            if (current == math.pi) or (current == math.pi/2):
                # update orientation
                orientation = self.get_new_orientation(current, x, y)
                # tell which direction to move 
                if (x < 0.0 and  y == 0.0) or (x == 0.0 and y < 0.0):
                    directions.append('left')
                else:
                    directions.append('right')
            # if facing 3pi/2 or 0 == 2pi, aka. 270 or 0 (where 0 == 360 bc of mod)
            elif (current == 3*math.pi/2) or (current == 0.0):
                orientation = self.get_new_orientation(current, -1*x, -1*y)
                if (x < 0.0 and  y == 0.0) or (x == 0.0 and y < 0.0):
                    directions.append('right')
                else:
                    directions.append('left')
            # update current orientation
            current = orientation
        return current, directions
            
    # return directions for movement for given edge, update current position
    def get_translation_directions(self, current, edge):
        # path to follow
        change_floor = self.graph[edge[0]]['floor'] != self.graph[edge[1]]['floor']
        if change_floor:
            goal = self.graph[edge[1]]['floor']
            current = (self.graph[edge[1]]['x'], self.graph[edge[1]]['y'])
            direction = f'take elevator/stairs to floor {goal}'
            return current, direction, change_floor
        # only give movement directions
        delta = (self.graph[edge[1]]['x'] - self.graph[edge[0]]['x'], 
                    self.graph[edge[1]]['y'] - self.graph[edge[0]]['y'])
        
        # update current position
        current = (current[0] + delta[0], current[1] + delta[1])

        # always pos cause neg is happening in turning
        move = max(abs(delta[0]), abs(delta[1]))
        direction = f'move forward {float(round(move, 2))} meters'
        return current, direction, change_floor

    # return directions from start position to goal position on graph using BFS
    # update current orientation and current position
    def get_directions(self, orientation, start, goal):
        # find path using BFS
        path = self.find_path(start, goal)
        edges = self.generate_edges(path)

        directions = []
        # current position and orientation in room/graph
        current_ori = orientation
        current_pos = (self.graph[start]['x'], self.graph[start]['y'])

        # for each edge in the path calculate directions
        for edge in edges:
            current_ori, turns = self.get_orientation_directions(current_ori, edge)
            current_pos, movement, elevator = self.get_translation_directions(current_pos, edge)
            # some positions may require more than one turn at a time (aka U-turns, etc.)
            for turn in turns:
                directions.append(turn)
            directions.append(movement)
            

            if elevator:
                current_ori = math.pi
                directions.append('turn to face exit of elevator/stairs')
                # print('turning to face the exit of the elevator assumed, changing current ori to pi')

        
        print(directions, current_ori, current_pos)
        # return directions in array, along with updated user info
        return directions, current_ori, current_pos
            
def main():
    # move this to parameters and init when a launch file is made
    try:
        rclpy.init()
        traverse = Graph()
        # have to feed in valid names on graph
        # robot_position : drake_desk : emily_desk : demo_table

        current_position = 'f2_p15' # 0.0:0.0 -- given it in name or in position???
        current_orientation = math.pi
        directions, current_orientation, current_position = traverse.get_directions(current_orientation, current_position, 'f2_p26')

        # rclpy.spin(traverse)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
