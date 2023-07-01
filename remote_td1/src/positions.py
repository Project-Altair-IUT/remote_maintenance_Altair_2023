#!/usr/bin/env python

## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

# Python 2/3 compatibility imports
from __future__ import print_function

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cos, radians

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

home_pose = geometry_msgs.msg.Pose()
home_pose.position.x = 0.2813
home_pose.position.y = 0.1124
home_pose.position.z = 0.3505
home_pose.orientation.x = 0
home_pose.orientation.y = 0.7070950753755282
home_pose.orientation.z = 0
home_pose.orientation.w = 0.7070950753755282

home_joint_goal = []
home_joint_goal.append(radians(0))
home_joint_goal.append(radians(-120))
home_joint_goal.append(radians(100))
home_joint_goal.append(radians(20))
home_joint_goal.append(radians(90))
home_joint_goal.append(radians(-90))

## the place on the left where IMU module is kept
imu_place = geometry_msgs.msg.Pose()
imu_place.position.x = 0.19132582738943238
imu_place.position.y = 0.2356817278349128
imu_place.position.z = 0.11938230955500673
imu_place.orientation.x = -0.7111337083932515
imu_place.orientation.y = 0.7029704048803254
imu_place.orientation.z = 0.010846756960616119
imu_place.orientation.w = 0.001951028623752082

## left side board where we have to stick the IMU module 
left_board = geometry_msgs.msg.Pose()
left_board.position.x = 0.29543432137091497
left_board.position.y = 0.17612193827029538
left_board.position.z = 0.4750929197533251
left_board.orientation.x = -0.19133129832659512
left_board.orientation.y = 0.6807265584059808
left_board.orientation.z = 0.19131910390326864
left_board.orientation.w = 0.6807354019316599

## top-left corner of center board

# position: 
#   x: 0.2814968119003112
#   y: 0.11257212723531237
#   z: 0.4887584644697226
# orientation: 
#   x: -7.721473549281385e-05
#   y: 0.7057183659811934
#   z: 0.0020666048089374714
#   w: 0.7084894572957896


## top-right corner of center board
# position: 
#   x: 0.30076148916894696
#   y: -0.09298244983999349
#   z: 0.4882709110543487
# orientation: 
#   x: -0.00012996906285835558
#   y: 0.7056967322937785
#   z: 0.0021181157667599115
#   w: 0.7085108458757173

## middle-right corner of center board
# position: 
#   x: 0.3014967745169874
#   y: -0.0924580888770981
#   z: 0.3102919355451365
# orientation: 
#   x: -0.0001536989547791067
#   y: 0.7057113227207551
#   z: 0.00211083518644061
#   w: 0.7084963300788292

