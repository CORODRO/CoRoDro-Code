#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Jan  4 16:46:45 2021
@author: Jasmine Rimani
"""

import math
import numpy as np
import rospy


# Utilities - from waypoint to x-y coordinates (lower left angle)
def ind2xy(ind, dx, dy, x_min, y_min, resolution):
    # rospy.loginfo('Inside ind2xy')

    i = ind % dx
    
    if i == 0:
        i = 0
        j = math.floor(ind / dx)
    else:
        j = math.floor(ind / dx)

    x = x_min + i*resolution
    y = y_min + j*resolution
    return x, y


# Utilities - from euler to quaternions
def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(
        yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(
        yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(
        yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(
        yaw / 2)

    q = [qx, qy, qz, qw]

    return q

# --> Cross Checked with Modeling and simulation of aerospace Vehicle dynamics, P.H. Zipfel
def quaternion_to_euler(x, y, z, w):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X = (math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = (math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = (math.atan2(t3, t4))


    return X, Y, Z
