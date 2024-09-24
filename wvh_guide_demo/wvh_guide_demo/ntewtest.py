#! /usr/bin/env python3

# # import dictionary for graph 
from collections import defaultdict 
import csv

import math
from math import atan2
from sensor_msgs.msg import JointState
import numpy as np


def get_orientation(current, delta_x ):
    # print(delta_x)
    # print(delta_y)
    # print(current)
    print(current - delta_x*math.pi/2)
    # print((current - delta_x*math.pi/2 - delta_y*math.pi/2)%(2*math.pi))
    return (current - delta_x*math.pi/2 )%(2*math.pi)

print( get_orientation((3*math.pi/2),  -1*-1))