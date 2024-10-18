#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from wvh_guide_demo_msgs.action import Directions
from parcs_stt_tts_msgs.action import TTS
from parcs_stt_tts_msgs.srv import Stop
import time
from parcs_stt_tts_msgs.action import Listen
from parcs_stt_tts_msgs.action import Recalibrate
from wvh_guide_demo_msgs.action import Location

class Chatbot(Node):

    def __init__(self):
        super().__init__('chatbot')

        self._directions_action_client = ActionClient(self, Directions, 'directions')
        self._locations_action_client = ActionClient(self, Location, 'location')
        
        self._tts_action_client = ActionClient(self, TTS, 'tts')
        self._listen_action_client = ActionClient(self, Listen, 'listen')
        self._recalibrate_action_client = ActionClient(self, Recalibrate, 'recalibrate')
        self._stop_srv_client = self.create_client(Stop, 'stop')

        while not self._stop_srv_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for stop service to become available...')
        self.get_logger().info("Stop service established!")

        self._goal_in_progress = False

        self.current_node = 'f1_p1'
        self.current_ori = [-1.0, 0.0]
        self.current_pos = []
        self.goal_node = 'f2_240'
        self.directions = ''
        self.transcript = ''
        
        self.send_listen_goal()
        self.send_directions_goal(self.current_ori, self.current_node, self.goal_node)

    '''FUNCTION CALLING LLMS'''
    def define_callable_functs(self):
        get_directions = {
            'name': 'send_directions_goal',
            'description': 'request directions from action server, using given starting position, orientation and a given goal location',
            'parameters': {
                'type': 'object',
                'properties': {
                    'ori': {
                        'type': 'float32[]',
                        'description': 'The users starting orientation'
                    },
                    'pos': {
                        'type': 'string',
                        'description': 'The users starting node position'
                    },
                    'goal': {
                        'type': 'string',
                        'description': 'The goal location for the user'
                    }
                },
                'required': ['ori', 'pos', 'goal'],
            }
        }
        


    '''DIRECTIONS'''
    def send_directions_goal(self, ori, pos, goal):
        goal_msg = Directions.Goal()
        goal_msg.orientation = ori
        goal_msg.position = pos
        goal_msg.goal = goal

        self.get_logger().info("Waiting for Directions action server...")
        self._directions_action_client.wait_for_server()
        self.get_logger().info("Directions action server found!")

        self._goal_in_progress = True
        self._send_goal_future = self._directions_action_client.send_goal_async(goal_msg, feedback_callback=self.directions_feedback_callback)
        self._send_goal_future.add_done_callback(self.directions_goal_response_callback)

    def directions_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Directions goal was rejected.')
            self._goal_in_progress = False
            return
        
        self.get_logger().info('Directions goal accepted, waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.directions_result_callback)

    def directions_result_callback(self, future):
        result = future.result().result

        self.current_node = self.goal_node
        self.goal_node = ''

        self.current_ori = result.orientation
        self.current_pos = result.position
        self.directions = result.directions

        self.get_logger().info('Directions result received.')
        self._goal_in_progress = False
        self.send_tts_goal(self.directions)

    def directions_feedback_callback(self, feedback_msg):
            self.get_logger().info(f'TTS feedback received: {feedback_msg}')

    '''TTS'''
    def send_tts_goal(self, msg):
        goal_msg = TTS.Goal()
        goal_msg.tts = msg

        self.get_logger().info("Waiting for TTS action server...")
        # self._tts_action_client.wait_for_server()
        if not self._tts_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available after waiting')
            return

        self.get_logger().info("TTS action server found!")

        self._goal_in_progress = True
        self._send_goal_future = self._tts_action_client.send_goal_async(goal_msg, feedback_callback=self.tts_feedback_callback)
        self._send_goal_future.add_done_callback(self.tts_goal_response_callback)

    def tts_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('TTS goal was rejected.')
            self._goal_in_progress = False
            return
        
        self.get_logger().info('TTS goal accepted, waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.tts_result_callback)

    def tts_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'TTS result received. Said: {result.msg}')
        self._goal_in_progress = False
    
    def tts_feedback_callback(self, feedback_msg):
        self.get_logger().info(f'TTS feedback received: {feedback_msg}')

    '''STOP'''
    def stop_serv_tts(self):
        request = Stop.Request()
        future = self._stop_srv_client.call_async(request)

        future.add_done_callback(self.stop_serv_resp)
    
    def stop_serv_resp(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Stop service response received. Success: {response.success}")
        except Exception as e:
            self.get_logger().error(f"Stop service call failed: {e}")

    '''LISTEN'''
    def send_listen_goal(self):
        goal_msg = Listen.Goal()

        self.get_logger().info("Waiting for listen action server...")
        self._listen_action_client.wait_for_server()
        self.get_logger().info("Listen action server found!")

        self._goal_in_progress = True
        self._send_goal_future = self._listen_action_client.send_goal_async(goal_msg, feedback_callback=self.listen_feedback_callback)
        self._send_goal_future.add_done_callback(self.listen_goal_response_callback)

    def listen_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Listen goal was rejected.')
            self._goal_in_progress = False
            return
        
        self.get_logger().info('Listen goal accepted, waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.listen_result_callback)

    def listen_result_callback(self, future):
        result = future.result().result
        self.transcript = result.transcript
        self.get_logger().info(f'Listen result received. Transcript: {result.transcript}')
        self._goal_in_progress = False
    
    def listen_feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Listen feedback received: {feedback_msg}')

    '''RECALIBRATION'''
    def send_recalibrate_goal(self):
        goal_msg = Recalibrate.Goal()

        self.get_logger().info("Waiting for recalibration action server...")
        self._recalibrate_action_client.wait_for_server()
        self.get_logger().info("Recalibration action server found!")

        self._goal_in_progress = True
        self._send_goal_future = self._recalibrate_action_client.send_goal_async(goal_msg, feedback_callback=self.recalibrate_feedback_callback)
        self._send_goal_future.add_done_callback(self.recalibrate_goal_response_callback)

    def recalibrate_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Recalibration goal was rejected.')
            self._goal_in_progress = False
            return
        
        self.get_logger().info('Recalibration goal accepted, waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.recalibrate_result_callback)

    def recalibrate_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Recalibration result received.\ndBFS: {result.dbfs}\nsilence_threshold: {result.threshold}')
        self._goal_in_progress = False
    
    def recalibrate_feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Recalibration feedback received: {feedback_msg}')
 

def main(args=None):
    rclpy.init(args=args)

    node = Chatbot()

    try: 
        rclpy.spin(node)
    
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()