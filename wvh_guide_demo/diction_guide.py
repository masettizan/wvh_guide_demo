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

    def get_orientation_directions(self, heading, edge):
        # find difference in starting node to end node of edge
        vector_u = np.array([self.graph[edge[0]]['x'], self.graph[edge[0]]['y']]) # vector a - where we are
        vector_v = np.array([self.graph[edge[1]]['x'], self.graph[edge[1]]['y']]) # vector b - where we are going
        
        # heading & head is in vector format
        head, theta, theta_direction = self.get_angle(heading, vector_u, vector_v)

        if theta == 0.0:
            return head, 'continue'
        
        direction = 'cw' if theta_direction == -1 else 'ccw'
        return head, f'turn {round(theta)} degrees {direction}'      

    def get_angle_direction(self, heading, goal):
        # these vectors are what were taking the dot product of in get_angle()
        cross = np.cross(heading, goal) 

        if cross > 0:
            return 1 # positive (+), ccw
        elif cross < 0:
            return -1 # negative (-), cw
        else:
            return 0 # colinear (cross == 0), when theta is 180 returns 0 : this is a problem :

    def get_angle(self, heading, u, v):
        # vector difference between 'a' and 'b' - hypotonuse
        goal_vector = v - u
        goal_norm = goal_vector/np.linalg.norm(goal_vector)

        heading_norm = heading/np.linalg.norm(heading)
        cos_theta = np.dot(heading_norm, goal_norm)

        theta = np.arccos(cos_theta)
        theta_direction = self.get_angle_direction(heading_norm, goal_norm)

        return goal_norm, np.degrees(theta), theta_direction

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
            current_ori, turn = self.get_orientation_directions(current_ori, edge)
            current_pos, movement, elevator = self.get_translation_directions(current_pos, edge)
            # some positions may require more than one turn at a time (aka U-turns, etc.)
            
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
        current_orientation = np.array([-1, 0])
        directions, current_orientation, current_position = traverse.get_directions(current_orientation, current_position, 'f2_p26')

        # rclpy.spin(traverse)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
