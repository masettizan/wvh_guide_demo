#! /usr/bin/env python3

import json
import openai

# Define functions with updated parameters and strict mode
def define_callable_functs():
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

'''FUNCTION CALLING LLMS'''
# Define the personality prompt according to the new requirements
def llm_parse_response(tools, user_input):
    personality = '''
    You are a friendly and helpful robot assistant designed to understand user intent and respond appropriately.
    For each user message, return a tuple with three elements: repeat, intent, and next_speech.

    - repeat: A boolean value indicating whether to continue the conversation ({repeat} for yes, {not repeat} for no).
    - intent: A string value identifying the main intent of the user's message, such as "{intent}".
    - next_speech: A response string that addresses the user's message directly.
    '''

    response = openai.chat.completions.create(
        model="gpt-4",
        messages=[
            {"role": "system", "content": personality},
            {"role": "user", "content": user_input},
        ],
        tools=tools,
        tool_choice="auto"
    )
    
    response_msg = response.choices[0].message

    print(response_msg)

def organize_llm_response(response):
    if response.content is not None:
        try:
            repeat, intent, next_speech = ast.literal_eval(response.content)
        except SyntaxError:
            repeat = True
            intent = 'Request'
            next_speech = response.content
    # a function may be getting called
    else:
        repeat = True
        intent = 'function'
        next_speech = ''

    if response.tool_calls is not None:
        funct = response.tool_calls[0] #its a list

        funct_call = json.loads(funct.function.arguments)
        next_speech = json.dumps(funct_call)

        intent = funct.function.name
    
    return repeat, intent, next_speech

import ast
# Initialize tools and test with a user input
tools = define_callable_functs()
# llm_parse_response(tools, 'can you navigate me')
string1 = '(True, "NavigationRequest", "Sure, I can help you with that. Could you please tell me where you would like to go?")\n'
str2 = 'hi how are you, i am'
try:
    hi , hello , bwa= ast.literal_eval(string1)
except SyntaxError:
    print(str2)