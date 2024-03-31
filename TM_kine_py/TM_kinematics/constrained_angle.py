'''
Code originally adapted from: https://github.com/dbddqy/visual_kinematics
'''
import numpy as np
from math import pi

# ================== constrain angle between -pi and pi
def simplify_angle(angle):
    while angle > pi:
        angle -= 2 * pi

    while angle < -pi:
        angle += 2 * pi

    return angle


# ================== constrain angles[n, ] between -pi and pi
def simplify_angles(angles):
    for i in range(angles.shape[0]): # extract each angle in the first (and only) column
        angles[i] = simplify_angle(angles[i])

    return angles
