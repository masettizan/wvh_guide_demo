#! /usr/bin/env python3

import ast
import threading

import openai
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
        # Define Action Client/Server Connection
        self._directions_action_client = ActionClient(self, Directions, 'directions')
        self._directions_action_client.wait_for_server()
        self._navigation_action_client = ActionClient(self, Navigation, 'navigation')
        self._navigation_action_client.wait_for_server()
        
        self.get_logger().info('starting tts')
        self._tts_action_client = ActionClient(self, TTS, 'tts')
        self._tts_action_client.wait_for_server()
        self._stop_srv_client = self.create_client(Stop, 'stop')
        self._stop_srv_client.wait_for_service(timeout_sec=1.0)
        self.get_logger().info('tts server found')

        self.get_logger().info('starting stt')
        self._stt_action_client = ActionClient(self, Listen, 'listen')
        self._recalibrate_action_client = ActionClient(self, Recalibrate, 'recalibrate')
        self._stt_action_client.wait_for_server()
        self.get_logger().info('stt server found')  

        # define functions for llm
        self.tools = self.define_callable_functs()
        self.chatbot_history = []

        self.current_node = 'f1_p1'
        self.current_ori = [-1.0, 0.0]
        self.current_pos = []
        self.goal_node = ''
        self.directions = ''
        self.transcript = ''

    '''FUNCTION CALLING LLMS'''
    # Define functions with updated parameters and strict mode
    def define_callable_functs(self):
        get_directions = {
            'name': 'send_directions_goal',
            'description': 'Request to get directions to a goal from the directions action server, using given starting position, orientation, and a given goal location.',
            'parameters': {
                'type': 'object',
                'properties': {
                    'goal': {
                        'type': 'string',
                        'description': 'The goal location for the user'
                    }
                },
                'required': ['goal'],
                'additionalProperties': False
            },
            'strict': False
        }

        get_navigated = {
            'name': 'send_navigation_goal',
            'description': 'Request to be brought and driven to a goal from the navigation action server, using a goal name given in the building West Village H.',
            'parameters': {
                'type': 'object',
                'properties': {
                    'goal': {
                        'type': 'string',
                        'description': 'The goal location to be accompanied to'
                    }
                },
                'required': ['goal'],
                'additionalProperties': False
            },
            'strict': False
        }

        tools = [
            {
                "type": "function",
                "function": get_directions
            },
            {
                "type": "function",
                "function": get_navigated
            }
        ]

        return tools

   
    # Interpret and organize llm result
    def organize_llm_response(self, response):
        if response.content is not None:
            try:
                repeat, next_speech = ast.literal_eval(response.content)
            except SyntaxError:
                repeat = True
                next_speech = response.content
        # a function may be getting called
        else:
            repeat = True
            next_speech = ''

        if response.tool_calls is not None:
            next_speech = response.tool_calls[0] #its a list

        return repeat, next_speech 
    
    # Define the personality prompt according to the new requirements
    def llm_parse_response(self, user_input):
        personality = '''
        You are a friendly and helpful robot assistant designed to understand user intent and respond appropriately.
        For each user message, return a tuple with two elements: repeat, and next_speech.

        - repeat: A boolean value indicating whether to continue the conversation (True for yes, False for no).
        - next_speech: A response string that addresses the user's message directly.

        You can both give instructions and guide users.
        '''
        
        self.chatbot_history.append({"role": "user", "content": user_input})

        response = openai.chat.completions.create(
            model="gpt-4o-mini",
            messages=[{"role": "system", "content": personality}] + self.chatbot_history,
            tools=self.tools,
        )
        
        response_msg = response.choices[0].message
        result = self.organize_llm_response(response_msg)
        self.chatbot_history.append({"role": "assistant", "content": result})

        return result

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
        
        return result.directions

    def directions_feedback_callback(self, feedback_msg):
            self.get_logger().info(f'Directions feedback received: {feedback_msg}')

    '''TTS'''
    def send_tts_goal(self, msg):
        goal_msg = TTS.Goal()
        goal_msg.tts = msg

        future = self._tts_action_client.send_goal_async(goal_msg, feedback_callback=self.tts_feedback_callback)
        # wait for the send_goal future to be done
        rclpy.spin_until_future_complete(self, future)
        # now we setup the loop for the results
        goal_future = future.result().get_result_async()

        #blocking call
        while not goal_future.done():
            rclpy.spin_until_future_complete(self, goal_future, timeout_sec=0.5)
        
        result = goal_future.result().result
        self.get_logger().info(f'TTS result received. Said: {result.msg}')

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

        future = self._stt_action_client.send_goal_async(goal_msg, feedback_callback=self.listen_feedback_callback)
        
        #wait for it to be done
        rclpy.spin_until_future_complete(self, future)
        goal_future = future.result().get_result_async()

        #blocking call
        while not goal_future.done():
            self.get_logger().info("waiting")
            rclpy.spin_until_future_complete(self, goal_future, timeout_sec=0.5)
        
        result = goal_future.result().result
        self.get_logger().info(f'Listen result received. Transcript: {result.transcript}')

        return result.transcript
    
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
        
        future = self._navigation_action_client.send_goal_async(goal_msg, feedback_callback=self.navigation_feedback_callback)
        
        #wait for it to be done
        rclpy.spin_until_future_complete(self, future)
        goal_future = future.result().get_result_async()
        
        #blocking call
        while not goal_future.done():
            rclpy.spin_until_future_complete(self, goal_future, timeout_sec=0.5)
        
        result = goal_future.result().result
        self.current_node = self.goal_node
        self.goal_node = ''

        return result.success

    def navigation_feedback_callback(self, feedback_msg):
            self.get_logger().info(f'Navigation feedback received: {feedback_msg}')

    '''INTERACTION'''
    def interaction(self):
        repeat = True
        self.send_tts_goal('how can I help you today?')


        while repeat:
            #dialoge
            response = self.send_listen_goal()
            #given response, parse it
            repeat, next_speech = self.llm_parse_response(response)
            
            if isinstance(next_speech, str):
                self.send_tts_goal(next_speech)
            else:
                args = json.loads(next_speech)
                goal = args['goal']
                function = args['name']

                if function == 'send_navigation_goal':
                    self.send_tts_goal(f'Sure, I can help you get to the {goal}')
                    success = self.send_navigation_goal(args['goal'])

                    if success:
                        self.send_tts_goal('We have arrived!')
                    else:
                        self.send_tts_goal("It seems we're having trouble navigating to your goal.")
                elif function == 'send_directions_goal':
                    self.send_tts_goal(f'Sure, I can give directions to the {goal}')
                    transcript = self.send_directions_goal(args['goal'])
                    self.send_tts_goal(transcript)

            self.send_tts_goal('Is there anything else I can help you with?')

def main(args=None):
    rclpy.init(args=args)

    node = Chatbot()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=10)
    executor.add_node(node)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    node.interaction()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()