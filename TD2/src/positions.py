#!/usr/bin/env python

## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

# Python 2/3 compatibility imports
from __future__ import print_function

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

## to rotate right
yaw_right = []
yaw_right.append(radians(-65))
yaw_right.append(radians(-120))
yaw_right.append(radians(100))
yaw_right.append(radians(20))
yaw_right.append(radians(90))
yaw_right.append(radians(-90))

## to look at inspection box area
inspection_box_area = geometry_msgs.msg.Pose()
inspection_box_area.position.x = 0.24807737376585776
inspection_box_area.position.y = -0.20104866798487891
inspection_box_area.position.z = 0.24797795913858736
inspection_box_area.orientation.x = 0.34741413088099216
inspection_box_area.orientation.y = 0.7047604986268203
inspection_box_area.orientation.z = -0.27260072809419333
inspection_box_area.orientation.w = 0.5552521087596123

## to look at inspection panel cover
inspection_panel_cover = geometry_msgs.msg.Pose()
inspection_panel_cover.position.x = 0.294911746470652
inspection_panel_cover.position.y = -0.20711669365835217
inspection_panel_cover.position.z = 0.3256490954478124
inspection_panel_cover.orientation.x = 0.294824287633354
inspection_panel_cover.orientation.y = 0.8423609818663809
inspection_panel_cover.orientation.z = -0.13854445825260434
inspection_panel_cover.orientation.w = 0.4293157913914703

## to look at cover storage area
cover_placement_area = geometry_msgs.msg.Pose()
cover_placement_area.position.x = 0.20018393327402137
cover_placement_area.position.y = -0.20693527041203508
cover_placement_area.position.z = 0.1565074799711708
cover_placement_area.orientation.x = 0.5304871735542791
cover_placement_area.orientation.y = 0.8326623047085541
cover_placement_area.orientation.z = -0.08535956120602305
cover_placement_area.orientation.w = 0.13405443044728707

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

left_board_joint = []
left_board_joint.append(radians(23))
left_board_joint.append(radians(-125))
left_board_joint.append(radians(106))
left_board_joint.append(radians(7))
left_board_joint.append(radians(90))
left_board_joint.append(radians(-90))




## center of the switch board
switch_panel_center = geometry_msgs.msg.Pose()
switch_panel_center.position.x = 0.4736415714452577 - (12.3/100)
switch_panel_center.position.y = 0.00035292609177118 
switch_panel_center.position.z = 0.40547853586886806 - (5.5/100)
switch_panel_center.orientation.x = -7.721473549281385e-05
switch_panel_center.orientation.y = 0.7057183659811934
switch_panel_center.orientation.z = 0.0020666048089374714
switch_panel_center.orientation.w = 0.7084894572957896

switch_panel_center_joint = [0 for _ in range(6)]
switch_panel_center_joint[0] = radians(-49)
switch_panel_center_joint[1] = radians(-116)
switch_panel_center_joint[2] = radians(99)
switch_panel_center_joint[3] = radians(17)
switch_panel_center_joint[4] = radians(40)
switch_panel_center_joint[5] = radians(-90)

top_left_center_joint_goal = [0 for _ in range(6)]
top_left_center_joint_goal[0] = 0
top_left_center_joint_goal[1] = radians(-88)
top_left_center_joint_goal[2] = radians(39)
top_left_center_joint_goal[3] = radians(49)
top_left_center_joint_goal[4] = radians(90)
top_left_center_joint_goal[5] = radians(-90)

# center pose of switch board, optimal for swich press
switch_center_pose = geometry_msgs.msg.Pose()
switch_center_pose.position.x = 0.3242394783080311
switch_center_pose.position.y = 0
switch_center_pose.position.z = 0.37312493268311087
switch_center_pose.orientation.x = 0
switch_center_pose.orientation.y = 0.7070950753755282
switch_center_pose.orientation.z = 0
switch_center_pose.orientation.w = 0.7070950753755282