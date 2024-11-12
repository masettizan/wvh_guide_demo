#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from wvh_guide_demo_msgs.action import Directions, Navigation
from parcs_stt_tts_msgs.action import TTS
from parcs_stt_tts_msgs.srv import Stop
import time
import json
from parcs_stt_tts_msgs.action import Listen
from parcs_stt_tts_msgs.action import Recalibrate
# from wvh_guide_demo_msgs.action import Location

class Chatbot(Node):

    def __init__(self):
        super().__init__('chatbot')

        self._directions_action_client = ActionClient(self, Directions, 'directions')
        self._navigation_action_client = ActionClient(self, Navigation, 'navigation')
        # self._locations_action_client = ActionClient(self, Location, 'location')
        
        self._tts_action_client = ActionClient(self, TTS, 'tts')
        self._listen_action_client = ActionClient(self, Listen, 'listen')
        self._recalibrate_action_client = ActionClient(self, Recalibrate, 'recalibrate')
        self._stop_srv_client = self.create_client(Stop, 'stop')

        while not self._stop_srv_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for stop service to become available...')
        self.get_logger().info("Stop service established!")

        self._goal_in_progress = False

        self.define_callable_functs()
        self.get_logger().info(f"PLEASE {self.tools}")
        self.generate_response = True

        self.current_node = 'f1_p1'
        self.current_ori = [-1.0, 0.0]
        self.current_pos = []
        self.goal_node = ''
        self.directions = ''
        self.transcript = ''
        
        # self.send_listen_goal()
        self.interaction()
        # self.send_tts_goal('give me directions to the exit')
        # self.send_directions_goal("bathroom")

    '''FUNCTION CALLING LLMS'''
    def define_callable_functs(self):
        get_directions = {
            'name': 'send_directions_goal',
            'description': 'request directions from action server, using given starting position, orientation and a given goal location',
            'parameters': {
                'type': 'object',
                'properties': {
                    'goal': {
                        'type': 'string',
                        'description': 'The goal location for the user'
                    }
                },
                'required': ['goal'],
            }
        }

        get_navigated = {
            'name': 'send_navigation_goal',
            'description': 'request to be brought to a goal from the action server, using a goal name given',
            'parameters': {
                'type': 'object',
                'properties': {
                    'goal': {
                        'type': 'string',
                        'description': 'The goal location to be accompanied to'
                    }
                },
                'required': ['goal'],
            }
        }

        self.tools = [
            {
                "type": "function",
                "function": get_directions
            },
            {
                "type": "function",
                "function": get_navigated
            }
        ]

    '''DIRECTIONS'''
    def send_directions_goal(self, goal):
        goal_msg = Directions.Goal()
        goal_msg.orientation = self.current_ori
        goal_msg.position = self.current_node
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
        self._goal_in_progress = True
        self.generate_response = False
        self.send_tts_goal(self.directions)
        self._goal_in_progress = True

    def directions_feedback_callback(self, feedback_msg):
            self.get_logger().info(f'Directions feedback received: {feedback_msg}')

    '''TTS'''
    def send_tts_goal(self, msg):
        goal_msg = TTS.Goal()
        goal_msg.tts = msg
        goal_msg.tools = json.dumps(self.tools)
        goal_msg.generate_response = self.generate_response

        self.get_logger().info("Waiting for TTS action server...")
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
            self._goal_in_progress = True
            return
        
        self.get_logger().info('TTS goal accepted, waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.tts_result_callback)

    def tts_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'TTS result received. Said: {result.msg}')

        #function calling
        if result.tool_call != '':
            arguments = json.loads(result.tool_call)
            goal = arguments['goal']
            name = result.function_name

            if 'directions' in name:
                self.send_directions_goal(goal) 
            elif 'navigation' in name:
                self.send_navigation_goal(goal)  
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
        self.generate_response = False
        self.send_tts_goal('hm let me think on that')
        time.sleep(3)

        result = future.result().result
        self.transcript = result.transcript
        self.get_logger().info(f'Listen result received. Transcript: {result.transcript}')
        self._goal_in_progress = True
        
        self.generate_response = True
        self.send_tts_goal(self.transcript)
    
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
        self._goal_in_progress = True
    
    def recalibrate_feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Recalibration feedback received: {feedback_msg}')
 
    '''NAVIGATION'''
    def send_navigation_goal(self, goal):
        goal_msg = Navigation.Goal()
        goal_msg.goal = goal
        
        self.get_logger().info("Waiting for Navigation action server...")
        self._navigation_action_client.wait_for_server()
        self.get_logger().info("Navigation action server found!")

        self._goal_in_progress = True
        self._send_goal_future = self._navigation_action_client.send_goal_async(goal_msg, feedback_callback=self.navigation_feedback_callback)
        self._send_goal_future.add_done_callback(self.navigation_goal_response_callback)

    def navigation_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Navigation goal was rejected.')
            self._goal_in_progress = False
            return
        
        self.get_logger().info('Navigation goal accepted, waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.navigation_result_callback)
    
    def navigation_result_callback(self, future):
        result = future.result().result

        self.current_node = self.goal_node
        self.goal_node = ''

        if result.success:
            self.get_logger().info('Navigation result received.')
            
            self.generate_response = True
            self.send_tts_goal("ask if there is anything else you can help with")
            # self._goal_in_progress = False
        else:
            self.generate_response = True
            self.send_tts_goal("apologize for failure and ask if there is anything else you can help with")
            # self._goal_in_progress = False
    
    def navigation_feedback_callback(self, feedback_msg):
            self.get_logger().info(f'Navigation feedback received: {feedback_msg}')

    '''INTERACTION'''
    def interaction(self):
        self.generate_response = False
        repeat = True
        self.send_tts_goal('how can I help you today')


        while repeat:
            self.send_listen_goal()
            self.generate_response = False
            while self._goal_in_progress:
                continue
                # self.get_logger().info("witing")
            self.get_logger().info("out of while loop")
            self.send_tts_goal('is there anything else i can help with')
            self.transcript = ''
        
        #move from this function to inside code???
        #TTS - how can i help
        #STT
        #TTS - with function calling
        #TTS - is there anything else i can help with
        #STT
        #TTS - with function calling
        #TTS - is there anything else i can help with
        #...
        return


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