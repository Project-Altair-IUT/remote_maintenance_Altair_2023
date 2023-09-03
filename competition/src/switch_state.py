#!/usr/bin/env python3

import sys
import copy
import rospy
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cos, radians
from std_msgs.msg import String , Bool
from moveit_commander.conversions import pose_to_list

from altair_msgs.msg import AltairAruco
from erc_aruco_msg.srv import ErcAruco, ErcArucoRequest, ErcArucoResponse

from positions import home_joint_goal, top_left_center_joint_goal, \
                    yaw_left, imu_area, left_board_joint, \
                    yaw_right, inspection_box_area,inspection_panel_cover, cover_placement_area

from moveitInterface import MoveGroupInterface
from utils import Gripper, readFile, aruco_saver_caller, detect_enable

gripper = Gripper()

arm = MoveGroupInterface()

memo = dict()


switch_state_publisher1 = rospy.Publisher("/button1" , Bool , queue_size=10)
switch_state_publisher2 = rospy.Publisher("/button2" , Bool , queue_size=10)
switch_state_publisher3 = rospy.Publisher("/button3" , Bool , queue_size=10)
switch_state_publisher4 = rospy.Publisher("/button4" , Bool , queue_size=10)

rate = rospy.Rate(10)
def get_marker_positions():
    # Get the aruco positions
    try: 
        tags = rospy.wait_for_message('/project_altair/aruco_poses', AltairAruco, timeout=200)

        for idx, r in zip(tags.results_id, tags.results):
            memo[idx] = [
                r.translation.x,
                r.translation.y,
                r.translation.z,
            ]            
        print(memo)
    except rospy.ROSException as e:
        rospy.logwarn("Timeout while waiting for message: {}".format(e))
        memo = (0,0,0) * 10
        print(memo)

def check_marker(marker_id):
    valueX = abs(arm.get_current_pose().position.x - memo[marker_id].translation.x) 
    valueY = abs(arm.get_current_pose().position.y - memo[marker_id].translation.y)
    valueZ = arm.get_current_pose().position.z - memo[marker_id].translation.z  

    y_check = (valueY > 0.00) and (valueY < 0.035)
    z_check = (valueZ > -0.086) and (valueZ < -0.014)
    x_check = valueX < 0.117
    check = x_check and y_check and z_check
    if(check) :
        print(marker_id ," " ,valueX   ," " ,  valueY , " " , valueZ , " " , check)
    else:
        print(check)
    return check


def main():
    get_marker_positions()
    while True:
        switch_state_publisher1.publish(check_marker(1))
        switch_state_publisher2.publish(check_marker(2))
        switch_state_publisher3.publish(check_marker(3))
        switch_state_publisher4.publish(check_marker(4))
        rate.sleep()
        

if __name__ == '__main__':


    main()

    
