#! /usr/bin/env python3

import rclpy
import heapq

from rclpy.node import Node
from rclpy.action import ActionServer
import xml.etree.ElementTree as ET
import numpy as np
from wvh_guide_demo_msgs.action import Directions

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class Graph(Node):
    CONVERSION = 3/17

    def __init__(self):
        super().__init__('graph')
        self.graph = {}
        # fill in graph variable
        self.set_locations()
        self.directions_callback_group = MutuallyExclusiveCallbackGroup() # handles one callback at a time

        self._action_server = ActionServer(
            self,
            Directions,
            'directions',
            self.directions_callback,
            callback_group=self.directions_callback_group
        )
    
    def directions_callback(self, goal_handle):
        self.get_logger().info("Executing Directions goal...")

        self._goal_handle = goal_handle
        self.goal = goal_handle

        goal = self._goal_handle.request
        current_orientation = np.array(goal.orientation, dtype=np.float32)
        current_position = np.array(goal.position, dtype=np.float32)
        end_point = goal.goal

        result = Directions.Result()

        directions, end_orientation, end_position = self.get_directions(current_orientation, current_position, end_point)
        result.directions = ', '.join(self.simplify(directions))
        result.orientation = end_orientation
        result.position = end_position

        return result


    def set_locations(self):
        tree = ET.parse('/home/masettizan/ros2_ws/src/wvh_guide_demo/svg/WVH.svg')
        root = tree.getroot()
        self.graph = {}
        
        if list(root):
            for child in list(root):
                self.element_to_dict(child)

        self.elevators = ['f1_elevator', 'f2_elevator', 'f3_elevator', 'f4_elevator']

    def element_to_dict(self, element):
        attributes = element.attrib
        if attributes.get('id') is None:
            return
        
        element_dict = {}
        element_dict['x'] = float(attributes.get('x')) * self.CONVERSION
        element_dict['y'] = float(attributes.get('y')) * self.CONVERSION
        element_dict['neighbors'] = attributes.get('neighbors').split(',')
        element_dict['floor'] = int(attributes.get('floor'))
        
        self.graph[attributes.get('id')] = element_dict

    # generate edges for path through graph 
    def generate_edges(self, path): 
        edges = [] 
    
        for idx in range(len(path) - 1):
            node = path[idx]
            next_node = path[idx + 1]
            # if edge exists then append 
            edges.append((node, next_node)) 
        return edges 

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
            print('elevator')
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
                    if round(-1*sign*theta/30)%12 == 0:
                        continue
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
                    result.append(f'move forward {action} ft')
            else:
                result.append(action)

        return result

def main():
    # move this to parameters and init when a launch file is made
    try:
        rclpy.init()
        traverse = Graph()

        current_position = 'f1_p1' # given in name
        current_orientation = np.array([-1, 0])
        directions, current_orientation, current_position = traverse.get_directions(current_orientation, current_position, 'f2_240')
        result = ', '.join(traverse.simplify(directions))
        print(result)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
