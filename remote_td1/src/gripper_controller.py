#!/usr/bin/env python

import rospy
import time
import geometry_msgs
from math import pi, tau, dist, fabs, cos
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import String
from utils import Gripper


gripper = Gripper()

gripper.open()