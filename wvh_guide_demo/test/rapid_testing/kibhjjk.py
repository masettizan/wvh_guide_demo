#! /usr/bin/env python3
import numpy as np

def _get_angle_direction(heading, goal):
    # these vectors are what were taking the dot product of in get_angle()
    cross = np.cross(heading, goal)
    
    if cross >= 0:
        return 1  # positive (+), ccw
    else:
        return -1  # negative (-), cw

# potentially fixed code

def _get_angle(heading, u, v):
    # vector difference between 'a' and 'b' - hypotenuse
    goal_vector = v - u

    goal_norm = goal_vector / np.linalg.norm(goal_vector)
    heading_norm = heading / np.linalg.norm(heading)
    cos_theta = np.dot(heading_norm, goal_norm)
    theta_dir = np.sign(np.cross(heading_norm, goal_norm))
    theta = np.arccos(cos_theta) * theta_dir
    if theta == 0 and np.sum(goal_norm - heading_norm) != 0:
        theta = np.pi
    return np.degrees(theta)


# Let's think about this as if you are in cardinal directions
# East is (1, 0) and North is (0, 1)

# The rotation is counter-clockwise is positive

# You are facing North 
print(_get_angle(np.array([0, 1]), np.array([0, 0]), np.array([0, 1])))  # should be zero
print(_get_angle(np.array([0, 1]), np.array([0, 0]), np.array([1, 0])))  # -90 degrees
print(_get_angle(np.array([0, 1]), np.array([0, 0]), np.array([0, -1])))  # 180 degrees
print(_get_angle(np.array([0, 1]), np.array([0, 0]), np.array([0.00001, -1])))  # ~180 degrees (-negative)
print(_get_angle(np.array([0, 1]), np.array([0, 0]), np.array([-1, 0])))  # 90 degrees

# You are facing East
print("Heading 1,0")
print(_get_angle(np.array([1, 0]), np.array([0, 0]), np.array([0, 1])))  # 90 degrees
print(_get_angle(np.array([1, 0]), np.array([0, 0]), np.array([1, 0])))  # should be zero
print(_get_angle(np.array([1, 0]), np.array([0, 0]), np.array([0, -1])))  # -90 degrees
print(_get_angle(np.array([1, 0]), np.array([0, 0]), np.array([-1, 0])))  # 180 degrees 
print(_get_angle(np.array([1, 0]), np.array([0, 0]), np.array([-1, 0.0001])))  # ~180 degrees
