#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv2 import aruco
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tf import TransformListener

class MarkerSlam:
    def __init__(self):
        rospy.init_node('marker_slam')
        # Aruco marker
        self.marker_positions = {}

        