#! /usr/bin/env python3
import threading
import rclpy
import heapq
import rclpy.executors
from rclpy.node import Node
from rclpy.action import ActionServer
from wvh_guide_demo_msgs.action import Navigation
from rclpy.executors import MultiThreadedExecutor
from stretch_nav2.robot_navigator import BasicNavigator, TaskResult
import json
from geometry_msgs.msg import PoseStamped
import numpy as np
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.time import Time
from tf2_ros import TransformException
import time

# gives position information about base_link
class FrameListener(Node):
    def __init__(self):
        super().__init__('frame_listener')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.position = None #change none

        time_period = 0.1 #seconds
        self.timer = self.create_timer(time_period, self.on_timer)

    def on_timer(self):
        try:
            now = Time()
            position = self.buffer.lookup_transform(
                target_frame='map',
                source_frame='base_link', #switvhing 
                time=now,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            self.position = position

        except TransformException as ex:
            self.get_logger().info(
                f'Could not tranform "map" to "base_link": {ex}'
            )

    def get_position(self):
        total_time = 0.0
        while(self.position is None):
            if total_time >= 5:
                self.get_logger().error(
                    f'Could not tranform "map" to "base_link" and ran out of time'
                )
                return 
            total_time += 0.1
            time.sleep(.1)
        return self.position.transform
 
class Navigate(Node):

    def __init__(self):
        super().__init__('navigation')
        self._set_locations()

        self.robot_position = FrameListener()
        exe = rclpy.executors.MultiThreadedExecutor(num_threads=2)
        exe.add_node(self.robot_position)
        exe_thread = threading.Thread(target= exe.spin, daemon=True)
        exe_thread.start()

        self.nav = BasicNavigator()
        self.get_logger().info('\n\n\n\n\n GET POSE \n\n\n\n\n')
        #-19.754 -7.256 -2.917

        # self.current_pose = self._get_position(x=-20.0, y=-5.5)# self._get_position(x=0.0, y=0.0) # x, y, theta
        self.start_node = 'f1_robot_position'
        self.current_pose = self._get_position(x=-0.85, y=5.75)
        #TODO FIX THIS SHIT% NOPT WORKING
        # curr_pose = self.robot_position.get_position()
        # self.current_pose = self._get_position(x=curr_pose.translation.x, 
        #                                        y=curr_pose.translation.y, 
        #                                        z=curr_pose.rotation.z, 
        #                                        w=curr_pose.rotation.w)
 
        self.nav.setInitialPose(self.current_pose) #TODO
        self.get_logger().info('\n\n\n\n\n SET POSE \n\n\n\n\n')
        self.nav.waitUntilNav2Active()

        self._action_server = ActionServer(
            self,
            Navigation,
            'navigation',
            self.navigation_callback
        )

    def navigation_callback(self, goal_handle):
        self.get_logger().info("Executing Navigation goal...")

        feedback_msg = Navigation.Feedback()
        feedback_msg.current_x = self.current_pose.pose.position.x
        feedback_msg.current_y = self.current_pose.pose.position.y

        goal_name = goal_handle.request.goal
        result = Navigation.Result()

        goal_location = self.convert_to_point(goal_name)
        self.get_logger().info(f'Navigating to {goal_name} at {goal_location}')
        self.nav.goToPose(goal_location)
        # self.get_logger().info('\n\n\n\n\n GO TO POSE \n\n\n\n\n')
        while not self.nav.isTaskComplete():
            # self.get_logger().info('\n\n\n\n\n IN PROG \n\n\n\n\n')
            curr_pos = self.robot_position.get_position()

            feedback_msg.current_x = curr_pos.translation.x
            feedback_msg.current_y = curr_pos.translation.y

            goal_handle.publish_feedback(feedback_msg)

        if self.nav.getResult() == TaskResult.SUCCEEDED:
            result.success = True
            result.message = 'success'
        else:
            result.success = False
            result.message = 'failure'
        
        self.current_pose = goal_location

        return result

    def _set_locations(self):
        #TODO: switch based on device
        self.graph = {}
        self.elevators = ['f1_elevator', 'f2_elevator', 'f3_elevator', 'f4_elevator']
        
        with open("/home/hello-robot/ament_ws/src/wvh_guide_demo/svg/WVH.json", "r") as f: #with open("/home/hello-robot/ament_ws/src/wvh_guide_demo/svg/exp/EXP.json", "r") as f: #
            data = json.load(f)
        self.graph = data
    
    def convert_to_point(self, goal):
        options = []
        weights = []
        for key,value in self.graph.items():
            if value['type'] == '':
                pass
            if value['type'] == goal:
                options.append(key)

        for key in options:
            path, weight = self.find_path(self.start_node, key)
            weights.append(weight)

        idx = weights.index(min(weights))
        key_goal = options[idx]
        goal_pose = self._get_position(x=self.graph[key_goal]['real_x'], y=self.graph[key_goal]['real_y'])
        
        return goal_pose

    def _get_position(self, x=0.0, y=0.0, z=0.0, w=1.0):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.nav.get_clock().now().to_msg()

        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = float(z)
        pose.pose.orientation.w = float(w)

        return pose

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
        total_weight = 0

        heapq.heappush(queue, (0, start))
        path = {start: (None, 0)} # prev node, cost

        while queue:
            cost, node = heapq.heappop(queue)
            
            if node == goal:
                result = []
                while node is not None:
                    result.append(node)
                    node = path[node][0]
                return result[::-1], total_weight # return resvered path

            for neighbor in self.graph[node]['neighbors']:
                weight = cost + self._calculate_edge_cost(node, neighbor)

                if neighbor not in path or weight < path[neighbor][1]:
                    path[neighbor] = (node, weight)
                    total_weight += weight
                    heapq.heappush(queue, (weight, neighbor))
 

def main():
    # move this to parameters and init when a launch file is made
    try:
        rclpy.init()
        traverse = Navigate()
        executor = MultiThreadedExecutor()
        executor.add_node(traverse)

        executor.spin()
    except KeyboardInterrupt:
        pass
    executor.shutdown()
    traverse.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
