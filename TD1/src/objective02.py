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
    
    print(button_ids)

def scan_centre():
    arm.go_to_joint_state(top_left_center_joint_goal)
    # Do sweeps
    for row in range(4):
        # Go left/right
        print(row)
        dy = -0.2 if row % 2 == 0 else 0.2
        plan, fraction = arm.plan_cartesian_path((0, dy, 0))
        arm.execute_plan(plan)

        # if it's the last row no need to go further down
        if row == 3:
            break

        # Go down
        plan, fraction = arm.plan_cartesian_path((0, 0, -0.12))
        arm.execute_plan(plan)
    
    arm.go_home()

def get_marker_positions():
    # Get the aruco positions
    tags = rospy.wait_for_message('/project_altair/aruco_poses', AltairAruco, 20)

    # Take the 4 stated on parameter

    i = 0
    for idx, r in zip(tags.results_id, tags.results):
        print(idx, button_ids[i])
        if button_ids[i] == idx and i < 2:
            i += 1
            marker = Aruco_Marker()
            marker.id = idx
            marker.x = r.translation.x
            marker.y = r.translation.y
            marker.z = r.translation.z
            button_markers.append(marker)
            # print("\n\n\n\n\n\n\n")
            # print(marker.id)
            # print(marker.x)
            # print(marker.y)
            # print(marker.z)

    # print(button_markers)

def press_buttons():
    for marker in button_markers:
        # arm.press_switch(marker)
        print(marker.id)

    # arm.press_switch(button_markers[0])

def main():
    gripper.open()
    time.sleep(6)   # to make sure gripper closes before next moves
    arm.go_home()
    
    get_params()
    
    # scan_centre()

    gripper.close()
    time.sleep(6)   # to make sure gripper closes before next moves

    get_marker_positions()
    time.sleep(1)

    
    # for marker in button_markers:
    #     print("\n\n\n\n\n\n\n")
    #     print(marker.id)
    #     print(marker.x)
    #     print(marker.y)
    #     print(marker.z)

    marker = button_markers[0]
    print(marker.id)
    print(marker.x)
    print(marker.y)
    print(marker.z)

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = 0
    pose_goal.orientation.y = 0.7071068
    pose_goal.orientation.z = 0
    pose_goal.orientation.w = 0.7071068
    pose_goal.position.x = (marker.x - 0.15)
    pose_goal.position.y = 0.060428125084351414
    pose_goal.position.z = (marker.z - 0.055)
    arm.go_to_pose_goal(pose_goal)
    press_buttons()
    
if __name__ == '__main__':
    main()
    
