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

from positions import home_joint_goal, yaw_left, left_board, top_left_center_joint_goal, lid, lid_placement, cover_area, imu_area
from moveitInterface import MoveGroupInterface

arm = MoveGroupInterface()

def scan_left():
    #ensure we are at home
    arm.go_home()

    #shoulder pan joint yaw left
    arm.go_to_joint_state(yaw_left)

    #look at imu area
    arm.go_to_pose_goal(imu_area)

    #go back to previous state
    arm.go_to_joint_state(yaw_left)

    #look at left panel
    arm.go_to_pose_goal(left_board)

    #return to home position
    arm.go_home()


def plan_cartesian_path(move_group, delta):
    dx, dy, dz = delta
    waypoints = []

    wpose = arm.move_group.get_current_pose().pose

    wpose.position.x += dx
    wpose.position.y += dy
    wpose.position.z += dz

    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

def main():
    arm.move_group.go(home_joint_goal, wait=True)
    arm.move_group.stop()

    # Go to left panel
    arm.move_group.set_pose_target(left_board)
    arm.move_group.go(wait=True)
    arm.move_group.stop()
    arm.move_group.clear_pose_targets()

    # Go to top left of the center panel
    # for some reason the pose goal is not working as expected
    arm.move_group.go(top_left_center_joint_goal, wait=True)
    arm.move_group.stop()

    # Do a sweep
    for row in range(5):
        # Go left/right
        dy = -0.2 if row % 2 == 0 else 0.2
        plan, fraction = plan_cartesian_path(arm.move_group, (0, dy, 0));
        arm.move_group.execute(plan, wait=True)
        arm.move_group.stop()

        # if it's the last row no need to go further down
        if row == 4:
            break

        # Go down
        plan, fraction = plan_cartesian_path(arm.move_group, (0, 0, -0.1));
        arm.move_group.execute(plan, wait=True)
        arm.move_group.stop()

    # Check IMU area
    arm.move_group.set_pose_target(imu_area)
    arm.move_group.go(wait=True)
    arm.move_group.stop()
    arm.move_group.clear_pose_targets()

    # Check the cover area
    arm.move_group.set_pose_target(cover_area)
    arm.move_group.go(wait=True)
    arm.move_group.stop()
    arm.move_group.clear_pose_targets()

    # Go check the lid & lid placements
    arm.move_group.set_pose_target(lid)
    arm.move_group.go(wait=True)
    arm.move_group.stop()
    arm.move_group.clear_pose_targets()

    arm.move_group.set_pose_target(lid_placement)
    arm.move_group.go(wait=True)
    arm.move_group.stop()
    arm.move_group.clear_pose_targets()

    # Get the aruco positions
    tags = rospy.wait_for_message('/project_altair/aruco_poses', AltairAruco, 20)
    memo = dict()
    for idx, r in zip(tags.results_id, tags.results):
        memo[idx] = [
            r.translation.x,
            r.translation.y,
            r.translation.z,
        ]

    # Call the scorer
    rospy.wait_for_service('erc_aruco_score')
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
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
    
if __name__ == '__main__':
    main()
    # while True:
    #     inp = input("press q to quit")
    #     if inp == 'q':
    #         break
    #     time.sleep(3)

    # arm.go_home()
    # arm.go_to_joint_state(yaw_left)

    # scan_left()

