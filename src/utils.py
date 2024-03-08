#!/usr/bin/env python3
# Python file to contain scripts for utility functions using for different nodes

import math
import numpy as np
import cv2


def get_limits(color, huelimit):
    '''
    Convert BGR color to HSV to get the limit range for a given color in the BGR cylinder
    '''
    c = np.uint8([[color]])  # BGR values
    hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)

    hue = hsvC[0][0][0]  # Get the hue value

    # Handle red hue wrap-around
    if hue >= 165:  # Upper limit for divided red hue
        lowerLimit = np.array([hue - huelimit, 100, 100], dtype=np.uint8)
        upperLimit = np.array([180, 255, 255], dtype=np.uint8)
    elif hue <= 15:  # Lower limit for divided red hue
        lowerLimit = np.array([0, 100, 100], dtype=np.uint8)
        upperLimit = np.array([hue + huelimit, 255, 255], dtype=np.uint8)
    else:
        lowerLimit = np.array([hue - huelimit, 100, 100], dtype=np.uint8)
        upperLimit = np.array([hue + huelimit, 255, 255], dtype=np.uint8)

    return lowerLimit, upperLimit

def findArea(x1, x2, y1, y2):
    return (x2 - x1) * (y2 - y1)

def findDistance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# Standardioze
degree_stdize = lambda x: (x + 360) % 360