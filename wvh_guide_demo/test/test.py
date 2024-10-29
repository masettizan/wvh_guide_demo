#! /usr/bin/env python3
from click import Choice
import openai
from pytest import Function
import rclpy

import numpy as np
from typing import List
import json

# arr = np.array([1.1, 1.2])
# arr_float32 = arr.astype(np.float32)
# print(arr, type(arr))
# # Convert to Python list
# arr = arr_float32.tolist()
# print(arr, type(arr))
# List[Dict[str, str]]


# arr = ChatCompletionMessageToolCall(id='call_ZFZ5z2DsUrWpig2RX8cCZkO1', function=Function(arguments='{"goal":"bathroom"}', name='send_directions_goal'), type='function')

# tool_call = arr.choices[0].message.tool_calls[0]
# arguments = json.loads(tool_call['function']['arguments'])
# print(arguments)

import json

# Sample dictionary
data = {
    "name": "John",
    "age": 30,
    "city": "New York"
}

# Save the dictionary to a JSON file
with open("/home/hello-robot/ament_ws/src/wvh_guide_demo/svg/WVH.json", "r") as f:
    d = json.load(f)
    print(d, type(d))