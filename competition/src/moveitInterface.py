#!/usr/bin/env python

## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

# Python 2/3 compatibility imports
from __future__ import print_function

import sys
import copy
import rospy
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cos, radians

from std_msgs.msg import String, Bool
from moveit_commander.conversions import pose_to_list

## END_SUB_TUTORIAL

#import positions from positions file
from positions import home_joint_goal, yaw_left, left_board, imu_area
from utils import all_close


class MoveGroupInterface(object):
    """MoveGroupInterface"""

    def __init__(self):
        super(MoveGroupInterface, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        self.robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        self.scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In ERC remote, the group is the gripper  
        ## joint of ur3, named "manipulator" so we set the group's name to "manipulator".
        ## This interface can be used to plan and execute motions:
        self.group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.move_group.set_max_velocity_scaling_factor(0.5)    
        self.move_group.set_planner_id("RRTConnectkConfigDefault")
        self.move_group.set_planning_time(30)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        self.planning_frame = self.move_group.get_planning_frame()
        print("============ Planning frame: %s" % self.planning_frame)

        # We can also print the name of the end-effector link for this group:
        self.eef_link = self.move_group.get_end_effector_link()
        print("============ End effector link: %s" % self.eef_link)

        # We can get a list of all the groups in the robot:
        self.group_names = self.robot.get_group_names()
        print("============ Available Planning Groups:", self.robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")

        self.initial_pose = self.move_group.get_current_pose()
        self.initial_joint_values = self.move_group.get_current_joint_values()

        ## END_SUB_TUTORIAL

    def go_to_joint_state(self, joint_goal):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
       
        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        time.sleep(1)
        ## END_SUB_TUTORIAL

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def get_joint_states(self):
        return self.move_group.get_current_joint_values()
    
    
    def go_to_pose_goal(self, pose_goal):
        
        print(self.robot.get_link_names("gripper"))
        print(self.scene.get_objects())
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        move_group.set_pose_target(pose_goal)
    
        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        move_group.clear_pose_targets()

        time.sleep(1)


        ## END_SUB_TUTORIAL
        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)
    
    def shift_pose_goal(self, axis , value):
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        move_group.shift_pose_target(axis , value)
    
        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        move_group.clear_pose_targets()

        time.sleep(1)


        ## END_SUB_TUTORIAL
        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        return True

    def go_home(self):
        self.go_to_joint_state(home_joint_goal)
        

    def plan_cartesian_path(self, delta, scale=1):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through. If executing  interactively in a
        ## Python shell, set scale = 1.0.
        ##
        dx, dy, dz = delta

        waypoints = []

        wpose = move_group.get_current_pose().pose
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

        ## END_SUB_TUTORIAL

    def show_current_pose(self):
        print(self.move_group.get_current_pose().pose)


    def get_current_pose(self):
        return self.move_group.get_current_pose().pose

    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        ## END_SUB_TUTORIAL

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)

        time.sleep(1)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL 
    
    def linear_move_to_pose (self, pose_goal):
        move_group = self.move_group

        next_point = [pose_goal]

        (plan, fraction) = move_group.compute_cartesian_path(
            next_point, 0.01, 0.0  # next point to go linear  # eef_step
        )  # jump_threshold

        self.execute_plan(plan)

    def press_switch(self, marker):
        pose_goal = geometry_msgs.msg.Pose()

        pose_goal.orientation.x = 0
        pose_goal.orientation.y = 0.7071068
        pose_goal.orientation.z = 0
        pose_goal.orientation.w = 0.7071068

        wait_time = 1.0

        dx1 = 12.1 / 100    #the lateral distance to keep from a button's root (also aruco's root) when it's openn
        dx2 = 11.5 / 100    #considering 6mm travel for the switch
        dz = 5.5 / 100      #the center of the button is 5.5cm below the center of aruco
        print("Calculated Switch", marker.id, "position")
        print(marker.pose.translation.x - 2.3 /100.0, marker.pose.translation.y, marker.pose.translation.z - dz)

        #hover_over switch        
        pose_goal.position.x = marker.pose.translation.x - dx1
        pose_goal.position.y = marker.pose.translation.y
        pose_goal.position.z = marker.pose.translation.z - dz
        self.linear_move_to_pose(pose_goal)

        print(f"Hovering over switch {marker.id}, {wait_time} seconds before pressing")
        time.sleep(wait_time)

        button_topic = f"/button{marker.id}"
        button_status = False
        t = 0

        #press switch
        while(button_status == False):
            t = t+1
            print(f'try: {t}')
            pose_goal.position.x = marker.pose.translation.x - dx2
            self.linear_move_to_pose(pose_goal)
            try: 
                button_status = rospy.wait_for_message(button_topic, Bool, timeout=3)
                print(f'from {button_topic} topic: {button_status}')
            except rospy.ROSException as e:
                print(e, button_topic)
                break


        print(f"Switch {marker.id} pressed, performing retraction")
        
        #retraction
        pose_goal.position.x = marker.pose.translation.x - dx1*t
        self.linear_move_to_pose(pose_goal)

