#!/usr/bin/env python3

## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

# Python 2/3 compatibility imports
from __future__ import print_function

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cos, radians

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

## END_SUB_TUTORIAL

#import positions from positions file
from positions import home_joint_goal, left_board, imu_area, yaw_left, yaw_right
from moveitInterface import MoveGroupInterface





def main():
    try:
        print("")
        print("----------------------------------------------------------")
        print("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")
        # input("============ Press `Enter` to begin the tutorial by setting up the moveit_commander ...")
        arm = MoveGroupInterface()
        arm.show_current_pose()

        # input("============ Press `Enter` to go to home pose ...")
        # arm.go_home()

        input(
            "============ Press `Enter` to execute a movement using a joint state goal ..."
        )
        arm.go_to_joint_state(yaw_right)

        # input("============ Press `Enter` to plan and display a Cartesian path ...")
        # cartesian_plan, fraction = arm.plan_cartesian_path()

        # input("============ Press `Enter` to execute a saved path ...")
        # arm.execute_plan(cartesian_plan)

        # input("============ Press `Enter` to execute a movement using a pose goal ...")
        # arm.go_to_pose_goal(imu_place)     
        
        
        # input("============ Press `Enter` to execute a movement using a pose goal ...")
        # arm.go_to_pose_goal(imu_area)   
        

        # input("============ Press `Enter` to go to home pose ...")
        # arm.go_home()

        # input(
        #     "============ Press `Enter` to plan and execute a path with an attached collision object ..."
        # )
        # cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
        # tutorial.execute_plan(cartesian_plan)

        # while True:
        #     input("============ Press `Enter` to see current pose ...")
        #     arm.show_current_pose()
            

        print("============ Python tutorial demo complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    # main()
    gripper_pub = rospy.Publisher('gripper_command', String, queue_size=10)
    gripper_pub.publish("close")

    # arm.go_to_joint_state(yaw_left)
