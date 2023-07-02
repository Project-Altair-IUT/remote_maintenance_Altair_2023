#!/usr/bin/env python3

import sys
import copy
import rospy
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cos, radians
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from altair_msgs.msg import AltairAruco
from erc_aruco_msg.srv import ErcAruco, ErcArucoRequest, ErcArucoResponse

from positions import home_joint_goal, top_left_center_joint_goal, \
                    yaw_left, imu_area, left_board_joint, \
                    yaw_right, inspection_box_area, cover_placement_area

from moveitInterface import MoveGroupInterface
from utils import Gripper, Aruco_Marker

gripper = Gripper()

arm = MoveGroupInterface()

button_ids = []

button_markers = []

def get_params():
    parameters = rospy.get_param('~tags', "0,0,0,0")
    for char in parameters:
        if char != ",":
            button_ids.append(int(char))
    
    # print(button_ids[0])

def get_marker_positions():
    # Get the aruco positions
    tags = rospy.wait_for_message('/project_altair/aruco_poses', AltairAruco, 20)

    # Take the 4 stated on parameter

    i = 0
    for idx, r in zip(tags.results_id, tags.results):
        if button_ids[i] == idx and i < 4:
            i += 1
            marker = Aruco_Marker()
            marker.id = idx
            marker.x = r.translation.x
            marker.y = r.translation.y
            marker.z = r.translation.z
            button_markers.append(marker)

    # print(button_markers)

def press_buttons():
    for marker in button_markers:
        arm.press_switch(marker)


def main():
    arm.go_home()
    
    gripper.close()
    time.sleep(1)   # to make sure gripper closes before next moves
    
    get_params()

    get_marker_positions()

    
    
if __name__ == '__main__':
    main()
    press_buttons()
    
