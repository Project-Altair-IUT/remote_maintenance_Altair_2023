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

## to rotate left
yaw_left = []
yaw_left.append(radians(35))
yaw_left.append(radians(-120))
yaw_left.append(radians(100))
yaw_left.append(radians(20))
yaw_left.append(radians(90))
yaw_left.append(radians(-90))

## the place on the left where IMU module is kept
imu_area = geometry_msgs.msg.Pose()
imu_area.position.x = 0.19132582738943238
imu_area.position.y = 0.2356817278349128
imu_area.position.z = 0.11938230955500673
imu_area.orientation.x = -0.7111337083932515
imu_area.orientation.y = 0.7029704048803254
imu_area.orientation.z = 0.010846756960616119
imu_area.orientation.w = 0.001951028623752082

## left side board where we have to stick the IMU module 
left_board = geometry_msgs.msg.Pose()
left_board.position.x = 0.2033890665790144
left_board.position.y = 0.24456285077225193
left_board.position.z = 0.3875504187763552
left_board.orientation.x = -0.11452377396802657
left_board.orientation.y = 0.6276256803624016
left_board.orientation.z = 0.15411417692619064
left_board.orientation.w = 0.7544661231732779


## top-left corner of center board
top_left_center = geometry_msgs.msg.Pose()
top_left_center.position.x = 0.2814968119003112
top_left_center.position.y = 0.11257212723531237
top_left_center.position.z = 0.4887584644697226
top_left_center.orientation.x = -7.721473549281385e-05
top_left_center.orientation.y = 0.7057183659811934
top_left_center.orientation.z = 0.0020666048089374714
top_left_center.orientation.w = 0.7084894572957896

top_left_center_joint_goal = [0 for _ in range(6)]
top_left_center_joint_goal[0] = 0
top_left_center_joint_goal[1] = radians(-88)
top_left_center_joint_goal[2] = radians(39)
top_left_center_joint_goal[3] = radians(49)
top_left_center_joint_goal[4] = radians(90)
top_left_center_joint_goal[5] = radians(-90)


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

# lid starting position
lid = geometry_msgs.msg.Pose()
lid.position.x = 0.2358072900697823
lid.position.y = -0.23415303785561903
lid.position.z = 0.2121063553827074
lid.orientation.x = 0.11669231495027149
lid.orientation.y = 0.6965663543474722
lid.orientation.z = -0.11427244757320609
lid.orientation.w = 0.6986558704743325

#position to keep the lid
lid_placement = geometry_msgs.msg.Pose()
lid_placement.position.x = 0.2869116211452445
lid_placement.position.y = -0.2031893729223361
lid_placement.position.z = 0.2679293893266063
lid_placement.orientation.x = 0.05766940464294197
lid_placement.orientation.y = 0.9188643663753105
lid_placement.orientation.z = -0.14686315274294895
lid_placement.orientation.w = 0.36165415847219146

# cover area
cover_area = geometry_msgs.msg.Pose()
cover_area.position.x = 0.14276348174207631
cover_area.position.y = -0.21525259000332297
cover_area.position.z = 0.3219434340662344
cover_area.orientation.x = -0.030230889882158038
cover_area.orientation.y = 0.998828681070058
cover_area.orientation.z = 0.011901195725936231
cover_area.orientation.w = 0.035856947849393805

# # imu
# imu_area = geometry_msgs.msg.Pose()
# imu_area.position.x = 0.13533445186563775
# imu_area.position.y = 0.24606431684893684
# imu_area.position.z = 0.11019034193242203
# imu_area.orientation.x = -0.6778411178011019
# imu_area.orientation.y = 0.7301153884438897
# imu_area.orientation.z = 0.013730398011894558
# imu_area.orientation.w = 0.08506639339906107