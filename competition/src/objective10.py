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


def main():
    gripper.open()
    arm.go_home()
    print("execution complete!")

if __name__ == '__main__':
    main()

    
