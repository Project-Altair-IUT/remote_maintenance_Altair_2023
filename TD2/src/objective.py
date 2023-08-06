#!/usr/bin/env python3

import rospy
import time
import geometry_msgs.msg
import tf2_ros
from math import pi, tau, dist, fabs, cos, radians
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from altair_msgs.msg import AltairAruco

from positions import home_joint_goal, top_left_center_joint_goal, switch_panel_center_joint, switch_panel_center,\
                    yaw_left, imu_area, left_board_joint, \
                    yaw_right, inspection_box_area, cover_placement_area

from moveitInterface import MoveGroupInterface
from utils import Gripper, Aruco_Marker, aruco_saver_caller, detect_enable
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

def second_look():
    print('giving a second look')
    pose_goal = geometry_msgs.msg.Pose()

    pose_goal.orientation.x = 0
    pose_goal.orientation.y = 0.7071068
    pose_goal.orientation.z = 0
    pose_goal.orientation.w = 0.7071068
    dx1 = 13.0 / 100
    dx2 = 11.5 / 100    #considering 6mm travel for the switch
    dz = 5.5 / 100      #the center of the button is 5.5cm below the center of aruco
        
    tfBuffer2 = tf2_ros.Buffer()
    listener2 = tf2_ros.TransformListener(tfBuffer2)
    detect_enable(True)
    for marker in to_press_button_markers:
        print(marker.pose)
        pose_goal.position = marker.pose.translation
        pose_goal.position.x = marker.pose.translation.x - dx1
        # pose_goal.position.z = marker.pose.translation.z - dz

        arm.linear_move_to_pose(pose_goal)
        
        try:
            tf_resultant = tfBuffer2.lookup_transform('base_link', 'fiducial_' + str(marker.id), rospy.Time(), rospy.Duration(1))
            
            marker.pose = tf_resultant.transform
            marker.numObservations += 1
            # MEMORY[idx] = marker    #saving the marker object in the dictionary
            print(f'updated: {marker.id}') 
            # print(tf_resultant.transform)

        except Exception as e:
            print(e)
    
    detect_enable(False)
    print('second look done')

def main():
    arm.go_home()
    get_params()

    aruco_saver_caller(True, to_press_button_ids)

    gripper.open()

    scan_centre(arm)

    aruco_saver_caller(False, [])

    get_marker_positions()

    second_look()

    arm.go_to_joint_state(switch_panel_center_joint)
    
    gripper.close()

    time.sleep(1)

    press_buttons()

    arm.go_home()
    
if __name__ == '__main__':
    main()

    
