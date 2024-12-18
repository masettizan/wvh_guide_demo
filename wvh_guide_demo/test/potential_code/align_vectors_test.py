#! /usr/bin/env python3

import numpy as np
import math

def rotate(a, b):
    # normalize vectors
    a_norm = a / np.linalg.norm(a) 
    b_norm = b / np.linalg.norm(b)

    # find angle between
    cos_theta = np.dot(a_norm, b_norm)
    print(cos_theta)
    theta = np.arccos(cos_theta) # / a_norm*b_norm is giving nan???
    print(theta)
    theta = np.degrees(theta)
    print("roatate", round(theta, 2))
    return theta

def angle(a, b):
    a_norm = np.linalg.norm(a)
    b_norm = np.linalg.norm(b)

    cos_theta = np.dot(a, b) / (a_norm * b_norm)
    cos_theta = np.clip(cos_theta, -1.0, 1.0)

    rad = np.arccos(cos_theta)
    deg = np.degrees(rad)
    print("angle", round(deg, 2))
    return deg

def compare(dir, a, b):
    new = b - a
    norm = new / np.linalg.norm(new) # current directino vector
    print(norm) #current
    dir_norm = dir / np.linalg.norm(dir)
    cosT = np.dot(dir_norm, norm)
    
    theta = np.arccos(cosT)
    print("angle",np.degrees(((theta + math.pi) % (2 * math.pi) - math.pi)))
    print(theta, np.degrees(theta)) #difference

def get_angle_direction(u, v):
    print(u, v)
    cross = np.cross(u,v)

    if cross > 0:
        return 1 # positive (+), ccw
    elif cross < 0:
        return -1 # negative (-), cw
    else:
        return 0 # colinear (cross == 0)

def get_angle(heading, a, b):
    new_heading = b - a
    new_head_norm = new_heading/np.linalg.norm(new_heading)

    heading_norm = heading/np.linalg.norm(heading)
    cos_theta = np.dot(heading_norm, new_head_norm)
   
    theta = np.arccos(cos_theta)

    return new_head_norm, np.degrees(theta)


def test():
    vector_a = np.array([1, 1])
    vector_b = np.array([1,5])
    # dir = get_dir(np.array([0,1]), vector_a)
    head = np.array([0,-1])
    # angle(vector_a,np.array([2, 2]))
    # rotate(vector_a, np.array([2, 2]))
    
    heading, angle = get_angle(head, vector_a, vector_b)
    dir = get_angle_direction(head, heading)
    print(head, angle, dir, dir*angle)
    # compare(dir, vector_a, vector_b)
    # dir = np.array([round(np.cos(curr),2), round(np.sin(curr),2)])
    # print(dir)
    print('\n')
    # assert(90.0 == angle(vector_a, np.array([1, 2])))
    # assert(90.0 == rotate(vector_a, np.array([1, 0])))
    # assert(180.0 == rotate(vector_a, np.array([0, 1])))
    # assert(0.0 == rotate(vector_a, np.array([2, 1])))
    # assert(45.0 == rotate(vector_a, np.array([0, 2])))
    # assert(45.0 == rotate(vector_a, np.array([0, 0])))
    # assert(45.0 == rotate(vector_a, np.array([2, 0])))
    # assert(45.0 == rotate(vector_a, np.array([2, 2])))



def cross():
    a = np.array([1, 1])
    b = np.array([1,1])
    print(a[0]*b[1] - a[1]*b[0])
    print(np.cross(a, b))

test()
print(type(np.array([1.1, 1.2])), type([1.1,1.2]))