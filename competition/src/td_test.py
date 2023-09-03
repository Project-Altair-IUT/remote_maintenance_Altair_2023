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
                    yaw_right, inspection_box_area, cover_placement_area ,lid_hold_pos

from moveitInterface import MoveGroupInterface
from utils import Gripper, Aruco_Marker
from objective01 import scan_centre

gripper = Gripper()

arm = MoveGroupInterface()

marker13 = Aruco_Marker(13)
marker13_pose = geometry_msgs.msg.Pose()

to_press_button_ids = []

to_press_button_markers = []

def get_params():
    parameters = rospy.get_param('~tags', "0,0,0,0")
    for char in parameters:
        if char != ",":
            to_press_button_ids.append(int(char))
    
    print(to_press_button_ids)

def get_marker13_position():
    # Get the aruco positions
    tags = rospy.wait_for_message('/project_altair/aruco_poses', AltairAruco, 20)

    # Take the 4 stated on parameter

    memo = dict()
    for idx, r in zip(tags.results_id, tags.results):
        if idx == 13: 
            memo[idx] = r

    print("Here is the memo " )
    print(memo)
    
    

    marker13.pose = memo[13]
    print("Marker13: ")
    print(marker13.pose)
    
    



def go_to_lid():
    arm.go_to_joint_state(yaw_right)
    #arm.go_to_pose_goal(inspection_box_area)

    current_goal_pos = geometry_msgs.msg.Pose()

    current_goal_pos.orientation = lid_hold_pos.orientation
    current_goal_pos.position = lid_hold_pos.position
    #hardcoded delta values from marker 13

    destx = 0.3470540509255145
    destY = -0.2805681637938778
    destZ = 0.2708827588802122
    sourceX = 0.37368479345604827
    sourceY = -0.2347042209560106
    sourceZ = 0.24037210237508502
    deltaX = destx -sourceX
    deltaY = destY - sourceY
    deltaZ = destZ - sourceZ

    # deltaPoint = geometry_msgs.msg.Transform()

    # deltaPoint.x = deltaX
    # deltaPoint.y = deltaY
    # deltaPoint.z = deltaZ
    current_goal_pos.position.x = marker13.pose.translation.x + deltaX -0.03
    current_goal_pos.position.y = marker13.pose.translation.y + deltaY +0.01
    current_goal_pos.position.z = marker13.pose.translation.z + deltaZ 
    print("Current Goal Pos :")
    print(current_goal_pos)
    
    arm.go_to_pose_goal(current_goal_pos)
    
    # current_goal_pos.position.x -= 0.03
    # current_goal_pos.position.y += 0.01
    # arm.go_to_pose_goal(current_goal_pos)
    gripper.semi_close()
    time.sleep(1)
    # current_goal_pos.position.z += 0.03
    # arm.go_to_pose_goal(current_goal_pos)

    arm.shift_pose_goal(2 , 0.03)

    # # current_joint = arm.get_joint_states()
    # # print(current_joint)
    # # print(home_joint_goal)
    # # current_joint[2] += radians(5)
    # # print(current_joint)

    # arm.go_to_joint_state(current_joint)
    time.sleep(6)

    


def main():
    arm.go_home()

    arm.show_current_pose()
    gripper.open()
    get_marker13_position()
    go_to_lid()
    
    gripper.close()
    
    
    time.sleep(1)

    
if __name__ == '__main__':
    main()

    
