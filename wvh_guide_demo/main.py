# Example usage
import safety_dialogue_agent.mm_agent_actual as mm_agent
import rclpy 
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.action import ActionClient
from rclpy.node import Node
from parcs_stt_tts_msgs.action import TTS, Listen, Recalibrate
from parcs_stt_tts_msgs.srv import Stop
import threading
from action_tutorials_interfaces.action import Fibonacci
import time
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
import hello_helpers.hello_misc as hm

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from PIL import Image as PIL_Image
import math 
import numpy as np

from safety_dialogue_agent.robot_navigator import BasicNavigator, TaskResult

import time
from datetime import datetime
# This function is a stripped down version of the code in
# https://github.com/matthew-brett/transforms3d/blob/f185e866ecccb66c545559bc9f2e19cb5025e0ab/transforms3d/euler.py
# Besides simplifying it, this version also inverts the order to return x,y,z,w, which is
# the way that ROS prefers it.
def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class JoyNode(Node):

    def __init__(self):
        super().__init__("joy_node")
        self.joy_sub = self.create_subscription(Joy, '/joy', self.callback, 1)
        self.joy_state = None
        self.b_pressed = False
    
    def callback(self, data):
        self.joy_state = data
        if self.joy_state.buttons[1] == 1:
            self.b_pressed = True
    
    def b_pressed(self):
        return self.b_pressed

    def get_joy_state(self):
        if self.joy_state is None:
            self.get_logger().info('waiting for joy')
            time.sleep(1)
        return self.joy_state

    def wait_for_a_button(self):
        self.get_logger().info('waiting for a button')
        while True:
            joy_state = self.get_joy_state()
            if joy_state.buttons[0] == 1:
                break
            time.sleep(0.1)


# convert image data to a png
class ImageConverter(Node):

    def __init__(self):
        super().__init__("image_converter")
        self.latest_image = None
        self.bridge = CvBridge()
        self.color_sub = self.create_subscription(Image, '/camera/color/image_raw', self.callback, 1)

    def callback(self,data):
        opencv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        color_coverted = cv2.cvtColor(opencv_image, cv2.COLOR_BGR2RGB) 
        # rotate this image clockwise  90 degres
        color_coverted = cv2.rotate(color_coverted, cv2.ROTATE_90_CLOCKWISE)
        self.latest_image = PIL_Image.fromarray(color_coverted)
        self.latest_image.save("latest_image.png")


    def get_image(self):
        if self.latest_image is None:
            self.get_logger().info('waiting for image')
            time.sleep(1)
        # save the image with the name "used_DDYYMMHHSS_image.png"
        self.latest_image.save(f"used_{time.strftime('%d%m%y%H%M%S')}_image.png")
        return self.latest_image

from std_srvs.srv import Trigger

class DialogueNode(Node):

    def __init__(self, img_node, joy_node, hello_node):
        super().__init__('dialogue_node')

        self.declare_parameter('mode', 'none')
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.tutorial_mode = False

        self.get_logger().info('starting tts')
        self._tts_action_client = ActionClient(self, TTS, 'tts')
        self._tts_action_client.wait_for_server()
        self._stop_srv_client = self.create_client(Stop, 'stop')
        self._stop_srv_client.wait_for_service()
        self.get_logger().info('tts server found')

        self.get_logger().info('starting stt')
        self._stt_action_client = ActionClient(self, Listen, 'listen')
        self._recalibrate_action_client = ActionClient(self, Recalibrate, 'recalibrate')
        self._stt_action_client.wait_for_server()
        self.get_logger().info('stt server found')        


        # stow service
        self.stow_service = self.create_client(Trigger, '/stow_the_robot')
        self.stow_service.wait_for_service()
        self.get_logger().info('stow service found')

        #LLM stuff
        if self.mode == "gpt":
            self.agent = mm_agent.MMSafetyGPT()
        elif self.mode == "new":
            self.agent = mm_agent.MMSafetyDISCAL()
        elif self.mode == 'tutorial':
            self.tutorial_mode = True
        else:
            raise ValueError(f"mode parameter must be either 'gpt', 'new', and 'tutorial'. Got {self.mode}")
        
        self.img_node = img_node
        self.joy_node = joy_node
        self.hello_node = hello_node
        if not self.tutorial_mode:
            self.agent.initiate()

        self.navigator = BasicNavigator()
        self.locations = {
            'start': {
                'x': 0.0,
                'y': 0.3,
                'theta': 0, # in radians
            },
            'camera': {
                'x': 1.2,  #1.2
                'y': 1.2, #1.2
                'theta': -1.1, #-1.1
            },
            'dialogue': {
                'x': 1.2,
                'y': 0.3,
                'theta': 0, 
            },
            'end': {
                'x': 0.0,
                'y': 0.3,
                'theta': 3.14, 
            },
        }

    def timer_callback(self):
        self.get_logger().info('timer callback')

    # return PoseStamped instance of pose given
    def get_pose(self, pos_x=0.0, pos_y=0.0, rot=0):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()

        pose.pose.position.x = float(pos_x)
        pose.pose.position.y = float(pos_y)
        pose.pose.position.z = 0.0

        quaternion = quaternion_from_euler(0, 0, rot)
        self.get_logger().info(f"quaternion: {quaternion}")

        pose.pose.orientation.x = float(quaternion[0])
        pose.pose.orientation.y = float(quaternion[1])
        pose.pose.orientation.z = float(quaternion[2])
        pose.pose.orientation.w = float(quaternion[3])
        return pose
    
        
    
    # go from current position to the target position
    def go_to_location(self, goal_location):

        to_pose = self.get_pose(
            pos_x=self.locations[goal_location]['x'], 
            pos_y=self.locations[goal_location]['y'],
            rot=self.locations[goal_location]['theta']
        )

        self.navigator.goToPose(to_pose)
        
        while not self.navigator.isTaskComplete():
            time.sleep(0.1)
            #rclpy.spin_once(self)
            if self.joy_node.get_joy_state().buttons[1] == 1 :
                self.get_logger().info('interrupting navigation')
                self.navigator.cancelTask()
                break


            #time.sleep(0.1)
        #     position = self.base_link_position.get_position()

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"yipee successfully got to position {goal_location}")
        else:
            self.get_logger().error('something went wrong -_-')

        # compare current base_link orientation with the target orientation
        # if not close enough, rotate the base_link

        



    def recalibrate(self):
        goal_msg = Recalibrate.Goal()
        self.get_logger().info('recalibrating stt...')
        future = self._recalibrate_action_client.send_goal_async(goal_msg)
        # wait for the send_goal future to be done
        rclpy.spin_until_future_complete(self, future)
        # now we setup the loop for the results
        goal_future = future.result().get_result_async()
        # loop goal future
        rclpy.spin_until_future_complete(self, goal_future)
        self.get_logger().info('stt recalibrated')

    def listen(self) -> str:
        goal_msg = Listen.Goal()
        future = self._stt_action_client.send_goal_async(goal_msg)
        # wait for the send_goal future to be done
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('finish listen future complete')
        # now we setup the loop for the results
        goal_future = future.result().get_result_async()
        while not goal_future.done():
            # rclpy.spin_once(self)
            rclpy.spin_until_future_complete(self, goal_future, timeout_sec=0.5)
            if self.joy_node.b_pressed:
                self.get_logger().info('interrupting')
                cancel_future = future.result().cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future)
                self.get_logger().info(f'listen cancel response: {cancel_future.result()}')
                return ""
        result = goal_future.result().result
        self.get_logger().info(f'listen:{result.transcript}')
        return result.transcript



    def speak(self, text, interruptable=True):
        goal_msg = TTS.Goal()
        goal_msg.tts = text

        self.get_logger().info(f'speak: {text}')
        future = self._tts_action_client.send_goal_async(goal_msg)
        # wait for the send_goal future to be done
        rclpy.spin_until_future_complete(self, future)
        # now we setup the loop for the results
        goal_future = future.result().get_result_async()

        while not goal_future.done():
            rclpy.spin_until_future_complete(self, goal_future, timeout_sec=0.5)
            if self.joy_node.b_pressed and interruptable:
                self.get_logger().info('interrupting')
                self._stop_srv_client.call_async(Stop.Request())
                break
        self.get_logger().info('speak done')
   
    def move_head(self, tilt, pan=0):
        self.hello_node.switch_to_position_mode()
        self.hello_node.move_to_pose({'joint_head_tilt': float(tilt), 'joint_head_pan': float(pan)}, blocking=True)
        self.hello_node.switch_to_navigation_mode()

    def run(self):

        # image_path = "/home/zhi/Dev/dialogue/Multimodal-Safety-Agent/input_images/0.jpg"
        # image = PIL_Image.open(image_path)

        # Open the file to record the conversation
        timestamp = datetime.now().strftime("%d%m%y%H%M%S")
        file_name = f"used_{timestamp}_conversation.txt"
        with open(file_name, 'w') as conversation_file:

            # stow the robot
            # future = self.stow_service.call_async(Trigger.Request())
            # rclpy.spin_until_future_complete(self, future)
            # recalibrate
            self.recalibrate()
            # self.listen()
            # start
            self.move_head(-0.5)
            self.joy_node.wait_for_a_button()

            self.go_to_location('start')
            time.sleep(1)
            self.joy_node.wait_for_a_button()
        
            self.go_to_location('camera')
            self.get_logger().info('getting image')
            image = self.img_node.get_image()
            self.get_logger().info('got image')
            if self.tutorial_mode:
                time.sleep(5)
                first_turn = "Your partner wanted me to remind you to turn off the lights before you leave. They also wanted me to ask you if you have paid the rent?"
            else:
                first_turn = self.agent.first_turn(image)
                self.get_logger().info(f"violation:{first_turn}")

            self.go_to_location('dialogue')
            self.move_head(0.3)

            self.speak('hi. ' + first_turn)
            conversation_file.write(f"robot: Hi. {first_turn}\n")

            if self.tutorial_mode:
                user_response = self.listen()
                conversation_file.write(f"user: {user_response}\n")
                self.speak('Great. I need to going back to my tasks now.', interruptable=False)
                conversation_file.write("robot: Great. I need to going back to my tasks now.\n")
            else:
                in_conv = True

                while (in_conv):
                    self.get_logger().info('listening...')
                    user_response = self.listen()
                    conversation_file.write(f"user: {user_response}\n")
                    self.move_head(0.3, -0.6)
                    if self.joy_node.b_pressed:
                        in_conv = False
                        break
                    self.get_logger().info(f'heard: {user_response}')
                    next_turn, in_conv = self.agent.next_turn(user_response)
                    self.move_head(0.3, 0)
                    self.speak(next_turn)
                    conversation_file.write(f"robot: {next_turn}\n")
                    
                    if self.joy_node.b_pressed:
                        in_conv = False
                
                time.sleep(1)
                self.move_head(0.3, 0)
                self.speak('Sorry. I need to going back to my tasks now.', interruptable=False)
                conversation_file.write("robot: Sorry. I need to going back to my tasks now.\n")
                
        self.move_head(0)
        self.go_to_location('start')


def main(args=None):
    rclpy.init(args=args)

    cam_node = ImageConverter()
    hello_node = hm.HelloNode.quick_create('hello')
    joy_node = JoyNode()
    node = DialogueNode(cam_node, joy_node, hello_node)

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=10)
    executor.add_node(node)
    executor.add_node(cam_node)
    executor.add_node(joy_node)
    #executor.add_node(hello_node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    node.run()

    # action_client = FibonacciActionClient()
    # future = action_client.send_goal(10)

    # rclpy.spin_until_future_complete(action_client, future)


if __name__ == "__main__":
    main()