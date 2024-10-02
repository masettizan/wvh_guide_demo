#! /usr/bin/env python3

import rclpy

import geojson
import heapq
import matplotlib.pyplot as plt 

import networkx as nx
import matplotlib.pyplot as plt
from rclpy.node import Node
import numpy as np
import time

class Graph(Node):
    
    def __init__(self):
        super().__init__('graph')
        self.graph = {}
        # fill in graph variable
        plt.ion()
        self.G = nx.Graph()
        self.set_locations()
        self.setup_map()
        # plt.show(block=False)


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
        
        self.elevators = ['f1_p7', 'f2_p1', 'f3_p1']
        # self.stairs = ['f1_p18', 'f1_p25', 'f2_p14', 'f2_p15', 'f3_p3', 'f3_p17']

    # generate edges for path through graph 
    def generate_edges(self, path): 
        edges = [] 
    
        for idx in range(len(path) - 1):
            node = path[idx]
            next_node = path[idx + 1]
            # if edge exists then append 
            edges.append((node, next_node)) 
        return edges 

    def add_node_to_graph(self, floor):
        nodes = []
        for id, data in self.graph.items():
            if data['floor'] == floor:
                self.G.add_node(id, 
                                pos=(data['x'], data['y']), 
                                floor=data['floor'], 
                                neighbors=data['neighbors']
                                )
                nodes.append(id)
        return nodes

    def add_edges_to_graph(self, floor):
        edges = []
        for id, data in self.graph.items():
            if data['floor'] == floor:
                for neighbor in data['neighbors']:
                    if id[:2] == neighbor[:2]:
                        self.G.add_edge(id, neighbor)
                        if (neighbor, id) not in edges:
                            edges.append((id, neighbor))
        return edges

    def setup_map(self):
        plt.figure(figsize=(10,8))
        self.draw_floor(1, 'purple')
        self.draw_floor(2, 'orange')
        self.draw_floor(3, 'red')

    def draw_floor(self, floor, color):
        nodes = self.add_node_to_graph(floor)
        edges = self.add_edges_to_graph(floor)
        pos = nx.get_node_attributes(self.G, 'pos')

        nx.draw(self.G, 
                pos, 
                nodelist=nodes, 
                edgelist=edges, 
                node_size=20, 
                node_color=color, 
                edge_color=color, 
                style='dashed', 
                with_labels=False
                )

    def draw_edge(self, edge):
        pos = nx.get_node_attributes(self.G, 'pos')
        nx.draw_networkx_edges(self.G, pos, edgelist=edge, width=2)

    def calculate_edge_cost(self, node_id, neighbor_id):
        node = self.graph[node_id]
        neighbor = self.graph[neighbor_id]

        if node['floor'] != neighbor['floor']:
            if node_id in self.elevators and neighbor_id in self.elevators:
                return 0 # motivate usage of elevators by making it low cost
            else: # only other option is stairs, seems reduntant to do elif
                return 10 # avg stair case for 10ft ceilings is 3m, assuming exp is 30ft ceilings

        node_coord = np.asarray((node['x'], node['y'])) 
        neighbor_coord = np.asarray((neighbor['x'], neighbor['y']))

        cost = node_coord - neighbor_coord # distance between coord vectors
        return np.sqrt(cost[0]**2 + cost[1]**2) # magnitude of cost vector

    # find path through dijkstra from start to goal in the graph
    def find_path(self, start, goal):
        queue = []

        heapq.heappush(queue, (0, start))
        path = {start: (None, 0)} # prev node, cost

        while queue:
            cost, node = heapq.heappop(queue)
            
            if node == goal:
                result = []
                while node is not None:
                    result.append(node)
                    node = path[node][0]
                return result[::-1] # return resvered path

            for neighbor in self.graph[node]['neighbors']:
                weight = cost + self.calculate_edge_cost(node, neighbor)

                if neighbor not in path or weight < path[neighbor][1]:
                    path[neighbor] = (node, weight)
                    heapq.heappush(queue, (weight, neighbor))
        
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
        vector_u = np.array([self.graph[edge[0]]['x'], self.graph[edge[0]]['y']]) # vector a - where we are
        vector_v = np.array([self.graph[edge[1]]['x'], self.graph[edge[1]]['y']]) # vector b - where we are going

        if self.graph[edge[0]]['floor'] != self.graph[edge[1]]['floor']:
            if edge[0] in self.elevators and edge[1] in self.elevators:
                transport = 'elevator' 
            else:
                transport = 'stairs'
            direction = f'take {transport} to floor {self.graph[edge[1]]["floor"]}'

            return vector_v, direction
        
        delta = vector_v - vector_u
        current = current + delta 

        move = np.sqrt(delta[0]**2 + delta[1]**2)
        direction = f'move forward {float(round(move, 2))} meters'
        return current, direction

    def get_directions(self, heading, start, goal):
        path = self.find_path(start, goal)
        edges = self.generate_edges(path)

        directions = []
        orientation = heading
        position = np.array([self.graph[start]['x'], self.graph[start]['y']])

        # for each edge in the path calculate directions
        for edge in edges:
            orientation, turn = self.get_orientation_directions(orientation, edge)
            position, movement = self.get_translation_directions(position, edge)
            
            directions.append(turn)
            directions.append(movement)
            
            if 'elevator' in movement: # assuming all elevators face the same direction: -1, 0 aka. pi
                orientation = np.array([-1, 0]) 
                directions.append('turn to face exit of elevator')
            elif 'stairs' in movement: # assuming all stair exits face the same direction: -1, 0 aka. pi
                orientation = np.array([-1, 0]) 
                directions.append('continue facing exit of stairs')

            self.draw_edge([edge])
            plt.pause(1)  # to have it plot out in real time and not immedietly close you need this line

        # return directions in array, along with updated user info
        return directions, orientation, position

# move this to parameters and init when a launch file is made           
def main():
    # if you want the graph to update in real time uncomment lines 21, 25, and 226
    #   if you want to keep graph open after the program is done running, uncomment lines 246 and 247
    # if you want the graph to be static, comment out lines 21, 25, 226, 246, 247. uncomment line 244
    try:
        rclpy.init()
        traverse = Graph()

        current_position = 'f1_p1' # given in name
        current_orientation = np.array([-1, 0])
        directions, current_orientation, current_position = traverse.get_directions(current_orientation, current_position, 'f3_p2')
        print(directions)
        #plt.show()

        # input("press enter to close graph")
        # plt.close()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()