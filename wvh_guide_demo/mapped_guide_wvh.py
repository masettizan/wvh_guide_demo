#! /usr/bin/env python3

import rclpy

import heapq
import matplotlib.pyplot as plt 

import networkx as nx
import matplotlib.pyplot as plt
from rclpy.node import Node
import numpy as np
import time
import xml.etree.ElementTree as ET
import matplotlib.image as mpimg

class Graph(Node):
    CONVERSION = 3/17   

    def __init__(self):
        super().__init__('graph')
        self.graph = {}
        # fill in graph variable
        
        self.G1 = nx.Graph()
        self.G2 = nx.Graph()
        self.G3 = nx.Graph()
        self.G4 = nx.Graph()
        self.scaling = [[2.63, 840, 2.63, 55],
              [1.05, 320, 1.05, 0],
              [.89, 240, .89, 30],
              [1.8, 545, 1.8, -820]]

        self.set_locations()
        self.setup_map()
        # plt.show(block=False)

    def set_locations(self):
        tree = ET.parse('/home/masettizan/ros2_ws/src/wvh_guide_demo/svg/WVH.svg')
        root = tree.getroot()
        self.graph = {}
        self.elevators = ['f1_elevator', 'f2_elevator', 'f3_elevator', 'f4_elevator']

        def element_to_dict(element, info):
            if element.attrib.get('id') is not None:
                element_dict = {}
                element_dict['x'] = float(element.attrib.get('x'))
                element_dict['y'] = float(element.attrib.get('y'))
                element_dict['neighbors'] = element.attrib.get('neighbors').split(',')
                element_dict['floor'] = int(element.attrib.get('floor'))
                
            
                info[element.attrib.get('id')] = element_dict
            return info
        
        if list(root):
            for child in list(root):
                self.graph = element_to_dict(child, self.graph)

    # generate edges for path through graph 
    def generate_edges(self, path): 
        edges = [] 
    
        for idx in range(len(path) - 1):
            node = path[idx]
            next_node = path[idx + 1]
            # if edge exists then append 
            edges.append((node, next_node)) 
        return edges 

    def add_node_to_graph(self, floor, g,size):
        nodes = []
        for id, data in self.graph.items():
            if data['floor'] == floor:
                print(size)
                g.add_node(id, pos=(data['x'] * size[0] - size[1], data['y']*size[2] +size[3]), floor=data['floor'], neighbors=data['neighbors'])
                nodes.append(id)
        return nodes

    def add_edges_to_graph(self, floor, g):
        edges = []
        for id, data in self.graph.items():
            if data['floor'] == floor:
                for neighbor in data['neighbors']:
                    if id[:2] == neighbor[:2]:
                        g.add_edge(id, neighbor)
                        if (neighbor, id) not in edges:
                            edges.append((id, neighbor))
        return edges
    
    def edge_location(self, floor, ax):
        if floor == 1:
            G=self.G1
            axe=ax[0,0]
        elif floor == 2:
            G=self.G2
            axe=ax[0,1]
        elif floor == 3:
            G=self.G3
            axe=ax[1,0]
        else:
            G=self.G4
            axe=ax[1,1]
        return G, axe
    
    def setup_map(self):
        colors = ['purple', 'orange', 'red', 'lightblue']
    
        plt.ion()
        fig, axes = plt.subplots(nrows=2, ncols=2, figsize = [10, 8])
        self.axe_hateme = axes
        for i in range(4):
            image = mpimg.imread(f'/home/masettizan/ros2_ws/src/wvh_guide_demo/svg/images/WVH_{i+1}.jpg')
            G, ax = self.edge_location(i+1, axes )
            ax.imshow(image)
            
            self.draw_floor(i+1 , colors[i],G, self.scaling[i],ax)

        plt.show(block=False)

    def draw_floor(self, floor, color,g,size, ax):
        nodes = self.add_node_to_graph(floor,g,size)
        edges = self.add_edges_to_graph(floor,g)
        pos = nx.get_node_attributes(g, 'pos')
        nx.draw(g, 
                pos, 
                nodelist=nodes, 
                edgelist=edges, 
                node_size=10, 
                node_color=color, 
                edge_color=color, 
                style='dashed', 
                with_labels=False, 
                ax=ax)

    def draw_edge(self, edge):
        u = edge[0][0]
        v = edge[0][1]
        if u[:2] == v[:2]:
            G, axe = self.edge_location(int(v[1:2]),self.axe_hateme)
            pos = nx.get_node_attributes(G, 'pos')
            nx.draw_networkx_edges(G, pos, edgelist=edge, width=2, ax=axe)
        plt.pause(1)

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
            return head, 0
        
        direction = 'cw' if theta_direction == -1 else 'ccw'
        return head, (round(theta), theta_direction)           

    def get_angle_direction(self, heading, goal):
        # these vectors are what were taking the dot product of in get_angle()
        cross = np.cross(heading, goal) 

        if cross >= 0:
            return 1 # positive (+), ccw
        else:
            return -1 # negative (-), cw
        
    def get_angle(self, heading, u, v):
        # vector difference between 'a' and 'b' - hypotonuse
        goal_vector = v - u

        if not np.any(goal_vector):
            return -1*heading, 180, 1
        
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
            direction = (transport, self.graph[edge[1]]["floor"])

            return vector_v, direction
        
        delta = vector_v - vector_u
        current = current + delta 

        move = np.sqrt(delta[0]**2 + delta[1]**2)
        direction = float(round(move, 2))
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
            
            directions.append(('rot', turn))
            directions.append(('move', movement))

            if isinstance(movement, tuple) and 'elevator' in movement[0]: # assuming all elevators face the same direction: -1, 0 aka. pi
                orientation = np.array([-1, 0]) 
                directions.append(('vert', 'turn to face exit of elevator'))
            elif isinstance(movement, tuple) and 'stairs' in movement[0]: # assuming all stair exits face the same direction: -1, 0 aka. pi
                orientation = np.array([-1, 0]) 
                directions.append(('vert', 'continue facing exit of stairs'))
            print(edge)
            self.draw_edge([edge])
            plt.pause(1)  # to have it plot out in real time and not immedietly close you need this line
            
        # return directions in array, along with updated user info
        return directions, orientation, position
    
    def simplify_rotation(self, directions):
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

    def simplify_translation(self, directions):
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

    def simplify(self, directions):
        dir = self.simplify_rotation(directions)
        dir = self.simplify_translation(dir)

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
        directions, current_orientation, current_position = traverse.get_directions(current_orientation, current_position, 'f2_240')
        result = ', '.join(traverse.simplify(directions))
        print(result)
        #plt.show()

        input("press enter to close graph")
        plt.close()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
