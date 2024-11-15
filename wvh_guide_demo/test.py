#! /usr/bin/env python3

import json
import openai
from openai.types.chat.chat_completion_message_tool_call import ChatCompletionMessageToolCall

# Define functions with updated parameters and strict mode
def define_callable_functs():
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

    return tools

'''FUNCTION CALLING LLMS'''
# Define the personality prompt according to the new requirements
def llm_parse_response(tools, user_input):
    personality = '''
    You are a friendly and helpful robot assistant designed to understand user intent and respond appropriately.
    For each user message, return a tuple with two elements: repeat, and next_speech.

    - repeat: A boolean value indicating whether to continue the conversation (True for yes, False for no).
    - next_speech: A response string that addresses the user's message directly.

    You can both give instructions and guide users.
    '''
    response = client.chat.completions.create(
        model="gpt-4o-mini",
        messages=[
            {"role": "system", "content": personality},
            {"role": "user", "content": user_input},
        ],
        tools=tools,
    )
    
    response_msg = response.choices[0].message

    print(response_msg)
    return organize_llm_response(response_msg)

def organize_llm_response(response):
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

    '''        #function calling
        if result.tool_call != '':
            arguments = json.loads(result.tool_call)
            goal = arguments['goal']
            name = result.function_name

            if 'directions' in name:
                self.send_directions_goal(goal) 
            elif 'navigation' in name:
                self.send_navigation_goal(goal)  
'''
import ast
# Initialize tools and test with a user input
tools = define_callable_functs()
# text = 'can you navigate me'
while True:
    text = input()
    print(llm_parse_response(tools, text))
    