# Example usage
import rclpy 
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.action import ActionClient
from rclpy.node import Node
import threading
from action_tutorials_interfaces.action import Fibonacci
import time
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
import hello_helpers.hello_misc as hm
from datetime import datetime
from std_srvs.srv import Trigger
import copy
from visualization_msgs.msg import MarkerArray
import random
import numpy as np

class InteractionNode(Node):

    def __init__(self, hello_node):
        super().__init__('interaction_node')
        self.hello_node = hello_node
        self.latest_face = []

        # create ros subscriber to face
        self.face_sub = self.create_subscription(MarkerArray, '/faces/marker_array', self.face_callback, 10)

        # Call on_timer function every second
        self.head_random_move_flag = True
        self.head_target = {
            'pan': 0.0,
            'tilt': 0.0
        }
        self.face_timer = self.create_timer(2.0, self.face_movement)

    def face_movement(self):
        std = {
            'pan': 0.03,
            'tilt': 0.01,

        }
        if self.head_random_move_flag:
            # add a bit noise to the target
            # make it normally distributed
            head_pan = np.random.normal(self.head_target['pan'], std['pan'])
            head_tilt = np.random.normal(self.head_target['tilt'], std['tilt'])
            # move head
            self.move_head(head_tilt, head_pan)

    def face_callback(self, msg):
        self.latest_face = msg.markers

    def wait_for_interactor(self, total_time = 1, timeout = -1):

        def face_close_enough(faces):
            return len(faces) > 0

        # move head up
        started = False
        start_time = datetime.now()
        while True and (timeout == -1 or (datetime.now() - start_time).total_seconds() < timeout):

            if face_close_enough(copy.deepcopy(self.latest_face)):
                if not started:
                    started = True
                    interact_start_time = datetime.now()
                else:
                    if (datetime.now() - interact_start_time).total_seconds() > total_time:
                        return True
        return False

   
    def move_head(self, tilt, pan=0):
        self.hello_node.switch_to_position_mode()
        self.hello_node.move_to_pose({'joint_head_tilt': float(tilt), 'joint_head_pan': float(pan)}, blocking=True)
        self.hello_node.switch_to_navigation_mode()

    def run(self):
        self.head_target['tilt'] = 0.3
        while True:
            self.get_logger().info(f'interactor{self.wait_for_interactor()}')
               

def main(args=None):
    rclpy.init(args=args)

    hello_node = hm.HelloNode.quick_create('hello')
    node = InteractionNode(hello_node)

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=10)
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    node.run()

    # action_client = FibonacciActionClient()
    # future = action_client.send_goal(10)

    # rclpy.spin_until_future_complete(action_client, future)


if __name__ == "__main__":
    main()