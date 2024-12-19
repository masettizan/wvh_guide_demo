#! /usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from wvh_guide_demo_msgs.action import Directions

class Chatbot(Node):

    def __init__(self, hello_node):
        super().__init__('chatbot')
        # hello node
        self.hello_node = hello_node

        # Define Action Client/Server Connection
        self._directions_action_client = ActionClient(self, Directions, 'directions')
        self._directions_action_client.wait_for_server()
        self.current_node = 'f1_robot_position'
        self.current_ori = [0.0, -1.0]

        '''DIRECTIONS'''
    def send_directions_goal(self, goal):
        goal_msg = Directions.Goal()
        goal_msg.orientation = self.current_ori
        goal_msg.position = self.current_node
        goal_msg.goal = goal

        future = self._directions_action_client.send_goal_async(goal_msg, feedback_callback=self.directions_feedback_callback)
        
        #wait for it to be done
        rclpy.spin_until_future_complete(self, future)
        goal_future = future.result().get_result_async()

        #blocking call
        while not goal_future.done():
            rclpy.spin_until_future_complete(self, goal_future, timeout_sec=0.5)
        
        result = goal_future.result().result
        self.current_node = self.goal_node
        self.goal_node = ''

        self.current_ori = result.orientation
        self.current_pos = result.position
        self.get_logger().info(result.directions)
        return result.directions
    
def main(args=None):
    rclpy.init(args=args)

    node = Chatbot()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=12)
    executor.add_node(node)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    node.send_directions_goal()
