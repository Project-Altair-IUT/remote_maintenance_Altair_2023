#!/usr/bin/env python3

import rospy
import time
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cos, radians
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from altair_msgs.msg import AltairAruco

from positions import home_joint_goal, top_left_center_joint_goal, switch_panel_center_joint, switch_panel_center,\
                    yaw_left, imu_area, left_board_joint, \
                    yaw_right, inspection_box_area, cover_placement_area

from moveitInterface import MoveGroupInterface
from utils import Gripper, Aruco_Marker
from objective01 import scan_centre

gripper = Gripper()

arm = MoveGroupInterface()

to_press_button_id = None

to_press_button_marker = None

def get_params():
    parameter = rospy.get_param('~tag', 1)
    global to_press_button_id 
    to_press_button_id = parameter
    
    print(f'gotta press button: {to_press_button_id}')

def get_marker_position():
    # Get the aruco positions
    tags = rospy.wait_for_message('/project_altair/aruco_poses', AltairAruco, 20)

    # Take the 4 stated on parameter

    memo = dict()
    for idx, r in zip(tags.results_id, tags.results):
        memo[idx] = r
    global to_press_button_id, to_press_button_marker
    idx = to_press_button_id
    marker = Aruco_Marker(idx)
    marker.pose = memo[idx]
    to_press_button_marker = marker
    # print(button_markers)

def press_button():
    global to_press_button_marker
    marker = to_press_button_marker
    arm.linear_move_to_pose(switch_panel_center)
    arm.press_switch(marker)
    print(marker.id)

    # arm.press_switch(button_markers[0])

def main():
    arm.go_home()

    arm.linear_move_to_pose(switch_panel_center)
    
    get_params()
    
    gripper.close()

    get_marker_position()
    time.sleep(1)

    press_button()
    
if __name__ == '__main__':
    main()

    
