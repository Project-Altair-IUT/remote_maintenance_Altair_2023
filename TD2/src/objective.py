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
from utils import Gripper, Aruco_Marker, aruco_saver_caller
from objective01 import scan_centre

gripper = Gripper()

arm = MoveGroupInterface()

to_press_button_ids = []

to_press_button_markers = []

def get_params():
    parameters = rospy.get_param('~tags', "0,0,0,0")
    for char in parameters:
        if char != ",":
            to_press_button_ids.append(int(char))
    
    print(to_press_button_ids)

def get_marker_positions():
    # Get the aruco positions
    tags = rospy.wait_for_message('/project_altair/aruco_poses', AltairAruco, 20)

    # Take the ones stated on parameter

    memo = dict()
    for idx, r in zip(tags.results_id, tags.results):
        memo[idx] = r

    for idx in to_press_button_ids:
        marker = Aruco_Marker(idx)
        try: 
            marker.pose = memo[idx]
            to_press_button_markers.append(marker)
            print(f'got position for marker: {marker.id}')
            print(marker.pose)

        except KeyError as e:
            print(f'could not get pose for marker with id = {idx}')

def press_buttons():
    for marker in to_press_button_markers:
        arm.press_switch(marker)
        arm.linear_move_to_pose(switch_panel_center)
        print(marker.id)


def main():
    arm.go_home()

    aruco_saver_caller(True, to_press_button_ids)

    gripper.open()

    scan_centre(arm)

    arm.go_to_joint_state(switch_panel_center_joint)
    
    get_params()
    
    gripper.close()

    get_marker_positions()
    time.sleep(1)

    press_buttons()

    arm.go_home()
    
if __name__ == '__main__':
    main()

    
