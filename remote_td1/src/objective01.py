#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cos, radians
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from altair_msgs.msg import AltairAruco
from erc_aruco_msg.srv import ErcAruco, ErcArucoRequest, ErcArucoResponse

from positions import home_joint_goal, left_board, top_left_center_joint_goal, lid, lid_placement, cover_area, imu_area

def plan_cartesian_path(move_group, delta):
    dx, dy, dz = delta
    waypoints = []

    wpose = move_group.get_current_pose().pose
    print('from here')
    print(wpose)
    wpose.position.x += dx
    wpose.position.y += dy
    wpose.position.z += dz
    print('going to right')
    print(wpose)
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
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_project_altair', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = 'manipulator'
    move_group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher(
        '/move_group/display_planned_path',
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20
    )

    planning_frame = move_group.get_planning_frame()
    print(f'Planning frame: {planning_frame}')

    eef_link = move_group.get_end_effector_link()
    print(f'Available planning groups:', robot.get_group_names())

    print('Printing robot state')
    print(robot.get_current_state())
    print()

    print(move_group.get_current_pose())

    # Home position
    move_group.go(home_joint_goal, wait=True)
    move_group.stop()

    # Go to left panel
    move_group.set_pose_target(left_board)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    # Go to top left of the center panel
    # for some reason the pose goal is not working as expected
    move_group.go(top_left_center_joint_goal, wait=True)
    move_group.stop()

    # Do a sweep
    for row in range(5):
        # Go left/right
        dy = -0.2 if row % 2 == 0 else 0.2
        plan, fraction = plan_cartesian_path(move_group, (0, dy, 0));
        move_group.execute(plan, wait=True)
        move_group.stop()

        # if it's the last row no need to go further down
        if row == 4:
            break

        # Go down
        plan, fraction = plan_cartesian_path(move_group, (0, 0, -0.1));
        move_group.execute(plan, wait=True)
        move_group.stop()

    # Check IMU area
    move_group.set_pose_target(imu_area)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    # Check the cover area
    move_group.set_pose_target(cover_area)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    # Go check the lid & lid placements
    move_group.set_pose_target(lid)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    move_group.set_pose_target(lid_placement)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    # Get the aruco positions
    tags = rospy.wait_for_message('/project_altair/aruco_poses', AltairAruco, 5)
    memo = dict()
    for r in tags.results:
        memo[r.id] = [
            r.pose.pose.position.x,
            r.pose.pose.position.y,
            r.pose.pose.position.z,
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
