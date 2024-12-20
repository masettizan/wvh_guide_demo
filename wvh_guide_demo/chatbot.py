#! /usr/bin/env python3

import ast
import threading

import numpy as np
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
import hello_helpers.hello_misc as hm
from visualization_msgs.msg import MarkerArray
from datetime import datetime
from pydantic import BaseModel
import copy

# from wvh_guide_demo_msgs.action import Location
class SentenceInterpretation(BaseModel):
    repeat: bool
    next_speech: str

class Chatbot(Node):

    def __init__(self, hello_node):
        super().__init__('chatbot')
        # hello node
        self.hello_node = hello_node

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

        # for face detection
        self.latest_face = []
        # create ros subscriber to face
        self.face_sub = self.create_subscription(MarkerArray, '/faces/marker_array', self.face_callback, 10)
        self.get_logger().info('added subscription to faces marker array')
        # Call on_timer function every second
        self.head_random_move_flag = True
        self.head_target = {
            'pan': 0.0,
            'tilt': 0.4
        } 
        self.face_timer = self.create_timer(2.0, self.face_movement)
        self.get_logger().info('finished initialization')

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
        # self.get_logger().info(f'messages: {msg}')
        # self.get_logger().info(f'{msg}')
        self.latest_face = msg.markers

    def wait_for_interactor(self, total_time = 1, timeout = -1):

        def face_close_enough(faces):
            # self.get_logger().info('face close eno0ugh')
            return len(faces) > 0

        # move head up
        started = False
        start_time = datetime.now()
        while True and (timeout == -1 or (datetime.now() - start_time).total_seconds() < timeout):
            rclpy.spin_once(self)
            # self.get_logger().info(f'{face_close_enough(copy.deepcopy(self.latest_face))}')
            # self.get_logger().info(f'{(self.latest_face)}')
            if face_close_enough(copy.deepcopy(self.latest_face)):
                # self.get_logger().info('we;re gere')
                if not started:
                    # self.get_logger().info('started face ckkse e3ogyug')
                    started = True
                    interact_start_time = datetime.now()
                else:
                    # self.get_logger().info(' not started face close ebnough')
                    if (datetime.now() - interact_start_time).total_seconds() > total_time:
                        return True
        return False

    def move_head(self, tilt, pan=0):
        self.hello_node.switch_to_position_mode()
        self.hello_node.move_to_pose({'joint_head_tilt': float(tilt), 'joint_head_pan': float(pan)}, blocking=True)
        self.hello_node.switch_to_navigation_mode()


    '''FUNCTION CALLING LLMS'''
    # Define functions with updated parameters and strict mode
    def define_callable_functs(self):
        get_directions = {
            'name': 'send_directions_goal',
            'description': 'Provide verbal directions to a goal location.',
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
            'description': 'Physically guide users to a goal name given in the building West Village H.',
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

        self.get_logger().info('callable functions defined')
        return tools

   
    # Interpret and organize llm result
    def organize_llm_response(self, response):
        if response.refusal:
            return True, "I have some trouble. Please try again", None
        
        function = None
        if len(response.tool_calls) > 0:
            function = response.tool_calls[0]
        
        if response.content is not None:
            try:
                string = response.content
                string.strip()
                output = json.loads(string)
                return output['repeat'], output['next_speech'], function
            except Exception as e:
                return True, "I have some trouble. Please try again", None

        if function is not None:
            return True, None, function #its a list
        return True, "Sorry I am confused, can you try again", None
        
    # Define the personality prompt according to the new requirements
    def llm_parse_response(self, user_input):
        self.personality = '''
        You are a friendly and helpful robot assistant designed to understand user intent and respond appropriately.
        For each user message, return a tuple with two elements: repeat, and next_speech.

        - repeat: A boolean value indicating whether to continue the conversation (True for yes, False for no).
        - next_speech: A response string that addresses the user's message directly.

        The output should be a JSON object that looks like: {"repeat":True, "next_speech": "speech"}

        You can both give instructions and guide users.
        '''
        
        self.chatbot_history.append({"role": "user", "content": user_input})

        response = openai.chat.completions.create(
            model="gpt-4o-mini",
            messages=[{"role": "system", "content": self.personality}] + self.chatbot_history,
            tools=self.tools,
            response_format=SentenceInterpretation
        )
        
        response_msg = response.choices[0].message
        self.get_logger().info(f'response : {response_msg}')
        result = self.organize_llm_response(response_msg)
        self.chatbot_history.append(response_msg)

        return result

    def complete_function_call(self, id, goal):
        self.get_logger().info(f'ID: {id}')
        function_result_message = {
            'role': 'tool',
            'content': json.dumps({
                'goal': goal,
                # variable_name: variable_value
            }),
            'tool_call_id': id
        }

        self.chatbot_history.append(function_result_message)

        #prepare payload
        completion_payload = {
            'model': 'gpt-4o-mini',
            'messages': [{"role": "system", "content": self.personality}] + self.chatbot_history,
        }
        self.get_logger().info(f'PAYLOPAd {completion_payload}')

        response = openai.chat.completions.create(
            model=completion_payload['model'],
            messages=completion_payload['messages']
        )
        self.get_logger().info(f'RESPONSE TO PAYLOPAF {response}')

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
        self.chatbot_history.append({"role": "assistant", "content": msg})

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

        future = self._recalibrate_action_client.send_goal_async(goal_msg, feedback_callback=self.recalibrate_feedback_callback)
        rclpy.spin_until_future_complete(self, future)
        goal_future = future.result().get_result_async()
        
        #blocking call
        while not goal_future.done():
            self.get_logger().info("recalibrating")
            rclpy.spin_until_future_complete(self, goal_future, timeout_sec=0.5)
        
        result = goal_future.result().result
        self.get_logger().info(f'Recalibration result received.\ndBFS: {result.dbfs}\nsilence_threshold: {result.threshold}')
    
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
        # commenting out for testing
        # self.send_recalibrate_goal()
        self.send_tts_goal('Dear Percy, I am ready to go.')
        self.head_random_move_flag = True
        self.head_target['tilt'] = 0.3
        self.get_logger().info('wait for interactor')
        while self.wait_for_interactor():
            
            repeat = True
            self.send_tts_goal('how can I help you today?')
            while repeat:
                #dialoge
                response = self.send_listen_goal()
                #given response, parse it
                repeat, next_speech, function = self.llm_parse_response(response)
                
                if next_speech and isinstance(next_speech, str):
                    self.send_tts_goal(next_speech)
                if function is not None: #function calling
                    goal = json.loads(function)['goal']
                    type = function.function.name

                    if type == 'send_navigation_goal':
                        self.complete_function_call(function.id, goal)
                        
                        self.send_tts_goal(f'Sure, I can help you get to the {goal}')
                        self.head_random_move_flag = False
                        self.move_head(pan=0, tilt=-0.3)
                        success = self.send_navigation_goal(goal)
                        self.head_target['tilt'] = 0.4
                        self.head_target['pan'] = 0.0
                        self.head_random_move_flag = True

                        if success:
                            self.send_tts_goal('We have arrived!')
                        else:
                            self.send_tts_goal("It seems we're having trouble navigating to your goal.")
                    elif type == 'send_directions_goal':
                        self.complete_function_call(function.id, goal)

                        self.send_tts_goal(f'Sure, I can give directions to the {goal}')
                        transcript = self.send_directions_goal(goal)
                        self.send_tts_goal(transcript)
        
        self.head_random_move_flag = True
        self.head_target['tilt'] = 0.4
        self.head_target['pan'] = 0.0
        time.sleep(15)


def main(args=None):
    #rclpy.init(args=args)

    hello_node = hm.HelloNode.quick_create('hello')
    node = Chatbot(hello_node)

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=12)
    executor.add_node(node)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    node.interaction()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()