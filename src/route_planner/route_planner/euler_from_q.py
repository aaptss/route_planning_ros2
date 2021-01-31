#!/usr/bin/env python3
import math
import numpy as np

def eu_from_q(x,y,z,w):
    num0 = +2 * (w * x + y * z)
    den0 = +1 - 2*(x * x + y * y)
    roll = math.atan2(num0, den0)

    num1 = +2 * (w * y - z * x)
    if num1 > +1:
        num1 = +1
    elif num1 < -1:
        num1 = -1
    else:
        pass
    pitch = math.asin(num1)

    num2 = +2 * (w * z + x* y)
    den2 = +1 - 2 * (y * y + z * z)
    yaw = math.atan2(num2,den2)

    return roll, pitch, yaw

def q_from_eu(roll,pitch,yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return qx, qy, qz, qw