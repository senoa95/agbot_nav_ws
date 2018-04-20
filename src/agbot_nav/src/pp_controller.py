#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Point32,Pose
import transforms3d as tf
import numpy as np


pi = 3.141592653589793238

class Point:

    def __init__(self,inputX = 0,inputY = 0,inputHeading = 0):
        self.x = inputX
        self.y = inputY
        self.heading = inputHeading
