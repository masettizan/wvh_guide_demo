#! /usr/bin/env python3

import rclpy
import heapq

from rclpy.node import Node
import xml.etree.ElementTree as ET
import numpy as np

class Graph(Node):
    CONVERSION = 3/17

    def __init__(self):
        super().__init__('graph')
        self.graph = {}
        # fill in graph variable
        self.set_locations()

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
    def _generate_edges(self, path): 
        edges = [] 
    
        for idx in range(len(path) - 1):
            node = path[idx]
            next_node = path[idx + 1]
            # if edge exists then append 
            edges.append((node, next_node)) 
        return edges 

    def _calculate_edge_cost(self, node_id, neighbor_id):
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
                weight = cost + self._calculate_edge_cost(node, neighbor)

                if neighbor not in path or weight < path[neighbor][1]:
                    path[neighbor] = (node, weight)
                    heapq.heappush(queue, (weight, neighbor))
        
    def _get_orientation_directions(self, heading, edge):
        # find difference in starting node to end node of edge
        vector_u = np.array([self.graph[edge[0]]['x'], self.graph[edge[0]]['y']]) # vector a - where we are
        vector_v = np.array([self.graph[edge[1]]['x'], self.graph[edge[1]]['y']]) # vector b - where we are going
        
        # heading & head is in vector format
        head, theta, theta_direction = self._get_angle(heading, vector_u, vector_v)

        if theta == 0.0:
            return head, 0
        
        direction = 'cw' if theta_direction == -1 else 'ccw'
        return head, (round(theta), theta_direction)        

    def _get_angle_direction(self, heading, goal):
        # these vectors are what were taking the dot product of in get_angle()
        cross = np.cross(heading, goal) 

        if cross >= 0:
            return 1 # positive (+), ccw
        else:
            return -1 # negative (-), cw
    
    def _get_angle(self, heading, u, v):
        # vector difference between 'a' and 'b' - hypotonuse
        goal_vector = v - u
        if not np.any(goal_vector):
            print('elevator')
            return -1*heading, 180, 1

        goal_norm = goal_vector/np.linalg.norm(goal_vector)

        heading_norm = heading/np.linalg.norm(heading)
        cos_theta = np.dot(heading_norm, goal_norm)

        theta = np.arccos(cos_theta)
        theta_direction = self._get_angle_direction(heading_norm, goal_norm)

        return goal_norm, np.degrees(theta), theta_direction

    # return directions for movement for given edge, update current position
    def _get_translation_directions(self, current, edge): 
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
        edges = self._generate_edges(path)

        directions = []
        orientation = heading
        position = np.array([self.graph[start]['x'], self.graph[start]['y']])

        # for each edge in the path calculate directions
        for edge in edges:
            orientation, turn = self._get_orientation_directions(orientation, edge)
            position, movement = self._get_translation_directions(position, edge)
            
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

    def _simplify_rotation(self, directions):
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

    def _simplify_translation(self, directions):
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
        dir = self._simplify_rotation(directions)
        dir = self._simplify_translation(dir)

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
