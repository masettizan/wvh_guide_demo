#! /usr/bin/env python3

# calculated through our map pgm, if map changes this amount will change
import rclpy
import heapq
import json

import numpy as np
from wvh_guide_demo_msgs.action import Directions

from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class Graph(Node):
    PX_TO_FT = 3/25 
    FT_TO_MIN = 4.7/60

    def __init__(self):
        super().__init__('graph')
        self.graph = self._set_locations()

        self.directions_callback_group = MutuallyExclusiveCallbackGroup() # handles one callback at a time
        self._dir_server = ActionServer( self, Directions, 'directions', self.directions_callback, callback_group=self.directions_callback_group)

    '''SETUP'''
    def _set_locations(self):
        with open("/home/masettizan/ros2_ws/src/wvh_guide_demo/svg/WVH.json", "r") as f: #with open("/home/hello-robot/ament_ws/src/wvh_guide_demo/svg/exp/EXP.json", "r") as f: #
            data = json.load(f)
        return data

    # calculate cost of edge between a node and it's neighbor
    def _calculate_cost(self, node_id, neighbor_id):
        node = self.graph[node_id]
        neighbor = self.graph[neighbor_id]

        if node['type'] == 'elevator' and neighbor['type'] == 'elevator':
            # motivate usage of elevators by making it low cost
            return 0
        
        # distance between coord vectors
        cost = np.asarray((node['x'], node['y'])) - np.asarray((neighbor['x'], neighbor['y']))
        return np.sqrt(cost[0]**2 + cost[1]**2) # magnitude

    # find best path from start to end using dijkstra's
    def _find_path(self, start, end):
        queue = []
        total_cost = 0
        
        heapq.heappush(queue, (start, 0))
        path = {start: (None, 0)} # prev node, cost

        while queue:
            node, cost = heapq.heappop(queue)
            
            if node == end:
                result = []
                while node is not None:
                    result.append(node)
                    node = path[node][0]
                return result[::-1], total_cost # return resvered path

            for neighbor in self.graph[node]['neighbors']:
                edge_cost = cost + self._calculate_cost(node, neighbor)

                if neighbor not in path or edge_cost < path[neighbor][1]:
                    path[neighbor] = (node, edge_cost)
                    total_cost += edge_cost
                    heapq.heappush(queue, (neighbor, edge_cost))

    def _find_node_by_type(self, start_node, type):
        potential = []
        cost = []

        # get potential nodes that could be the goal location
        for key, value in self.graph.items():
            if value['type'] == type:
                potential.append(key)

                # find the cost of each potential node
                path, weight = self._find_path(start_node, type)
                cost.append(weight)
        
        idx = cost.index(min(cost))
        return potential[idx]

    def _generate_edges(self, path):
        edges = []
        for idx in range(len(path) - 1):
            curr_node = path[idx]
            next_node = path[idx + 1]

            edges.append((curr_node, next_node))
        return edges

    '''ROTATION'''
    # return the amount to turn (in degrees) from the current direction being faced (aka. heading)
    def _get_angle(self, heading, u, v):
        goal_vector = v - u

        goal_norm = goal_vector/np.linalg.norm(goal_vector)
        heading_norm = heading/np.linalg.norm(heading)

        cos_theta = np.dot(heading_norm, goal_norm)
        theta_dir = np.sign(np.cross(heading_norm, goal_norm))

        theta = np.arccos(cos_theta) * theta_dir

        if theta == 0 and np.sum(goal_norm - heading_norm) != 0:
            theta = np.pi

        return np.degrees(theta)

    def get_rotation(self, heading, edge):
        # find difference from start node to end node
        vector_u = np.array([self.graph[edge[0]]['x'], self.graph[edge[0]]['y']])
        vector_v = np.array([self.graph[edge[1]]['x'], self.graph[edge[1]]['y']])

        theta = self._get_angle(heading, vector_u, vector_v) 

        # get new heading with theta in degrees
        curr_heading_rad = np.arctan2(heading[1], heading[0])
        new_heading = (np.degrees(curr_heading_rad) + theta) % 360
        
        return np.array([np.cos(np.radians(new_heading)), np.sin(np.radians(new_heading))]), round(theta)

    '''TRANSLATION'''
    def get_translation(self, curr_coords, edge):
        # find difference from start node to end node
        vector_u = np.array([self.graph[edge[0]]['x'], self.graph[edge[0]]['y']])
        vector_v = np.array([self.graph[edge[1]]['x'], self.graph[edge[1]]['y']])

        if self.graph[edge[0]]['floor'] != self.graph[edge[1]]['floor']:
            # take the {type} to the {floor} floor
            return vector_v, (self.graph[edge[0]]['type'], self.graph[edge[1]]['floor'])
        
        delta = vector_v - vector_u
        movement = np.sqrt(delta[0]**2 + delta[1]**2)
        return curr_coords + delta, float(round(movement, 2))
    
    '''COMBINE DIRECTIONS'''
    def compute_directions(self, heading, start, end):
        path, cost = self._find_path(start, end)
        edges = self._generate_edges(path)
        directions = []

        curr_position = np.array([self.graph[start]['x'], self.graph[start]['y']])
        for edge in edges:
            heading, theta = self.get_rotation(heading, edge)
            curr_position, movement = self.get_translation(curr_position, edge)

            directions.append(('rot', round((-1 * theta/30)%12))) # mult by -1 : bc the radian circle moves opposite
            if isinstance(movement, tuple):
                directions.append(('vert', f'take the {movement[0]} to floor {movement[1]}'))
            else:
                directions.append(('move', movement))

        return directions, heading, curr_position

    '''SIMPLIFICATION'''
    def _simplify_rotation(self, directions):
        idx = 0
        while idx < len(directions):
            type, action = directions[idx]
            if type == 'rot':     
                rot = self._clock_to_cardinal(action)
                if not rot or rot == 'forwards':
                    directions.pop(idx)
                else:
                    directions[idx] = (type, rot)
                    idx += 1
            else:
                idx += 1
        return directions

    def _combine_translation(self, directions):
        idx = 0
        while idx < len(directions) - 1:
            type, action = directions[idx]
            next_type, next_action = directions[idx + 1]

            if type == 'move' and next_type == 'move':
                action += next_action
                directions.pop(idx + 1)
                directions[idx] = (type, action)
            else:
                idx += 1
        return directions

    def _clock_to_cardinal(self, action):
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

    def simplify(self, directions):
        dir = self._simplify_rotation(directions)
        dir = self._combine_translation(dir)
        result = []

        for step in dir:
            type, action = step
            if type == 'rot':
                if action == 'backwards':
                    result.append(f'turn {action}')
                else:
                    result.append(f'turn to your {action}')
            elif type == 'move':
                result.append(f'move forward {round(action * self.PX_TO_FT * self.FT_TO_MIN, 1)} minutes')
            else:
                result.append(action)
        return result

    '''CALLBACK'''
    def directions_callback(self, goal_handle):
        self.get_logger().info("Executing Directions goal...")
        request = goal_handle.request

        curr_ori = np.array(request.orientation, dtype=np.float32)
        curr_pos = request.position
        goal = request.goal

        result = Directions.Result()     
        if goal in self.graph.keys():
            goal_node = goal
        else:
            goal_node = self._find_node_by_type(curr_pos, goal)
        
        directions, curr_ori, curr_pos = self.compute_directions(curr_ori, curr_pos, goal_node)
        result.directions = ', '.join(self.simplify(directions))
        result.orientation = curr_ori.astype(np.float32).tolist()
        result.position = curr_pos.astype(np.float32).tolist()
        
        return result

def main():
    rclpy.init()
    # move this to parameters and init when a launch file is made
    traverse = Graph()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(traverse)

    executor.spin()
    
    # directions, ori, pos = traverse.directions_callback(np.array([0, -1]), 'f1_robot_position', 'f1_p1')
    # print(directions, ori, pos)
        
if __name__ == '__main__':
    main()
