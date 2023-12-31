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
                    yaw_right, inspection_box_area,inspection_panel_cover, cover_placement_area

from moveitInterface import MoveGroupInterface
from utils import Gripper, readFile, aruco_saver_caller, detect_enable, Aruco_Marker

altair_sim = False

gripper = Gripper()

arm = MoveGroupInterface()

scanned_markers = []

def get_marker_positions():
    # Get the aruco positions
    tags = rospy.wait_for_message('/project_altair/aruco_poses', AltairAruco, 20)

    # Take the ones stated on parameter

    memo = dict()
    for idx, r in zip(tags.results_id, tags.results):
        memo[idx] = r

    for idx in range(1,10):
        marker = Aruco_Marker(idx)
        try: 
            marker.pose = memo[idx]
            scanned_markers.append(marker)
            print(f'got position for marker: {marker.id}')
            print(marker.pose)

        except KeyError as e:
            print(f'could not get pose for marker with id = {idx}')


def scan_left():
    #ensure we are at home
    # arm.go_home()

    #shoulder pan joint yaw left
    arm.go_to_joint_state(yaw_left)

    #look at imu area
    arm.go_to_pose_goal(imu_area)

    aruco_saver_caller(True, [10])
    time.sleep(3)
    aruco_saver_caller(False, [])
    
    #go back to previous state
    arm.go_to_joint_state(yaw_left)

    #look at left panel
    arm.go_to_joint_state(left_board_joint)
    aruco_saver_caller(True, [11])
    time.sleep(3)
    aruco_saver_caller(False, [])


    #go back to previous state
    arm.go_to_joint_state(yaw_left)

    #return to home position
    arm.go_home()

def scan_right():
    #ensure we are at home
    arm.go_home()

    #shoulder pan joint yaw right
    arm.go_to_joint_state(yaw_right)


    #look at planel cover storage area
    arm.go_to_pose_goal(cover_placement_area)

    aruco_saver_caller(True, [14])
    time.sleep(3)
    aruco_saver_caller(False, [])

    #go back to previous state
    arm.go_to_joint_state(yaw_right)

    #look at inspection panel area
    arm.go_to_pose_goal(inspection_box_area)

    aruco_saver_caller(True, [12])
    time.sleep(3)
    aruco_saver_caller(False, [])

    #look at inspection panel cover
    arm.go_to_pose_goal(inspection_panel_cover)

    aruco_saver_caller(True, [13])
    time.sleep(3)
    aruco_saver_caller(False, [])

    #go back to previous state
    arm.go_to_joint_state(yaw_right)

    #return to home position
    arm.go_home()


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
        
    for marker in scanned_markers:
        print(marker.pose)
        pose_goal.position = marker.pose.translation
        pose_goal.position.x = marker.pose.translation.x - dx1
        # pose_goal.position.z = marker.pose.translation.z - dz

        arm.linear_move_to_pose(pose_goal)
        
        aruco_saver_caller(True, [marker.id])
        time.sleep(3)
        aruco_saver_caller(False, [])
        
    
    print('second look done')

def scan_centre(arm):
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
    

def submit():
    # Get the aruco positions
    tags = rospy.wait_for_message('/project_altair/aruco_poses', AltairAruco, 20)
    # aruco_memory = readFile()
    memo = dict()
    for idx, r in zip(tags.results_id, tags.results):
        memo[idx] = [
            r.translation.x,
            r.translation.y,
            r.translation.z,
        ]
    print('got these:')
    print(memo)

    # Call the scorer
    rospy.wait_for_service('erc_aruco_score')
    print('waiting for scorer service')
    try:
        service_proxy = rospy.ServiceProxy('erc_aruco_score', ErcAruco)
        service_msg = ErcArucoRequest()
        service_msg.tag1=memo.get(1, [0, 0, 0])
        service_msg.tag2=memo.get(2, [0, 0, 0])
        service_msg.tag3=memo.get(3, [0, 0, 0])
        service_msg.tag4=memo.get(4, [0, 0, 0])
        service_msg.tag5=memo.get(5, [0, 0, 0])
        service_msg.tag6=memo.get(6, [0, 0, 0])
        service_msg.tag7=memo.get(7, [0, 0, 0])
        service_msg.tag8=memo.get(8, [0, 0, 0])
        service_msg.tag9=memo.get(9, [0, 0, 0])
        service_msg.tag10=memo.get(10, [0, 0, 0])
        service_msg.tag11=memo.get(11, [0, 0, 0])
        service_msg.tag12=memo.get(12, [0, 0, 0])
        service_msg.tag13=memo.get(13, [0, 0, 0])
        service_msg.tag14=memo.get(14, [0, 0, 0])
        print(service_msg)
        service_response = service_proxy(service_msg)
        print(f"Received score: {service_response.score}")
        if altair_sim:
            print(f"Corrected tags: {service_response.corrects}")

    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

def main():
    detect_enable(False)

    arm.go_home()
    gripper.open()
    
    scan_left()
    scan_right()

    aruco_saver_caller(True, [i for i in range (1,10)])
    scan_centre(arm)
    aruco_saver_caller(False, [])

    get_marker_positions()
    second_look()
    
    submit()
if __name__ == '__main__':
    arm.go_home()

    main()

    
