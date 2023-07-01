#!/usr/bin/env python
from __future__ import print_function
import os
import sys
import math
import copy
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True

class Marker():
    def __init__(self):
        self.id = None
        self.x = None 
        self.y = None 
        self.z = None 
        self.pan = None 
        self.tilt = None 
        self.roll = None 
        self.variance = None 
        self.numObservations = None 
        self.links = None

class Gripper():
    def __init__(self):
        self.gripper_pub = rospy.Publisher('gripper_command', String, queue_size=10)
        self.state = None
        self.sleep_time = 5.0
        # self.command = "close"

    def actuate(self, command):
        log_msg = "Gripper state : " + command
        rospy.loginfo(log_msg)
        self.state = command
        self.gripper_pub.publish(command)
        time.sleep(self.sleep_time) 
        return self.state

    def close(self):
        self.actuate("close")

    def semi_close(self):
        self.actuate("semi_close")

    def semi_open(self):
        self.actuate("semi_open")

    def open(self):
        self.actuate("open")


class MoveGroupPythonInteface(object):

    def __init__(self):

        super(MoveGroupPythonInteface, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.move_group.set_max_velocity_scaling_factor(0.5)    
        self.move_group.set_planner_id("RRTConnectkConfigDefault")
        self.move_group.set_planning_time(30)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()
        self.initial_pose = self.move_group.get_current_pose()
        self.initial_joint_values = self.move_group.get_current_joint_values()
        self.center = self.go_to_center()
        self.gripper0 = Gripper()
        self.gripper_state = self.gripper0.state
        self.gripper0.close()
        time.sleep(5.0)
        print("Gripper Closed")
        print(self.initial_pose)


    def print_current_state(self):
        print("Planning Frame: \n", self.planning_frame, '\n')
        print("End Effector Link: \n", self.eef_link, '\n')
        print("Available Groups: \n", self.group_names, '\n')
        #print("Current Robot State: \n", self.move_group.get_current_pose()._type, '\n', self.move_group.get_current_pose())
        #print("Current Robot State: \n", type(self.move_group.get_current_joint_values()), '\n', self.move_group.get_current_joint_values())

    def go_to_pose_goal(self,goal):
        self.move_group.set_goal_tolerance(0.1)
        self.move_group.set_pose_target(goal)

        #move_group.set_random_target()
        #print("Going to \n", goal._type,"\n", goal)


        ## Now, we call the planner to compute the plan and execute it.
        plan = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()

        self.move_group.set_goal_tolerance(0.01)
        self.move_group.set_pose_target(goal)
        ## Now, we call the planner to compute the plan and execute it.
        plan = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        self.move_group.clear_pose_targets()
            


    def plan_cartesian_path(self, scale=1):
        print("Doing sweep")

        waypoints = []


        wpose = self.move_group.get_current_pose().pose

        wpose.position.y -= scale * 0.08  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.y -= scale * 0.08  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints,0.01,0.0)  # waypoints to follow, eef_step,jump_threshold


        return plan, fraction

    def display_trajectory(self, plan):


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
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        self.display_trajectory_publisher.publish(display_trajectory);

        ## END_SUB_TUTORIAL


    def execute_plan(self, plan):

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        self.move_group.execute(plan, wait=True)



    def gofromtop_lefttoright(self):
        cartesian_plan, fraction = self.plan_cartesian_path()
        self.display_trajectory(cartesian_plan)
        self.execute_plan(cartesian_plan)
    def gofrombot_righttoleft(self):
        cartesian_plan, fraction = self.plan_cartesian_path(scale=-1.0)
        self.display_trajectory(cartesian_plan)
        self.execute_plan(cartesian_plan)

    def scan_center_panel(self):
        self.go_to_pose_goal(self.center)

        print("Arm at center")

        #exit()
        self.go_to_pose_goal(self.go_to_top_left())
        print("Going top left")

        self.gofromtop_lefttoright()

        self.go_to_pose_goal(self.go_to_top_right())
        print("Going top right")
        self.go_to_pose_goal(self.go_to_bot_right())
        print("Going bot right")    
        self.gofrombot_righttoleft()
        self.go_to_pose_goal(self.go_to_bot_left())
        print("Going bot left")

    def do_objective_1(self):
        """
        1.Scan the panel to localize its components
        """
        print("reseting to center position")
        #retract arm
        self.go_to_pose_goal(self.center)
        exit()
        self.scan_center_panel() # - > get switch poses
        print("reseting to center position")
        #retract arm
        self.go_to_pose_goal(self.center)
        print("Objective 1 Done")

    def press_switch(self,marker):
        print("Calculated Switch", marker.id, "position")
        print(marker.x - 2.0 /100.0, marker.y, marker.z - 0.065)
        #hover_over switch
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = 0.7071068
        pose_goal.orientation.y = 0.0
        pose_goal.orientation.z = 0.0
        pose_goal.orientation.w = 0.7071068
        pose_goal.position.x = marker.x - 10.0/100.0
        pose_goal.position.y = marker.y
        pose_goal.position.z = marker.z - 0.065 #6.5 cm
        self.go_to_pose_goal(pose_goal)
        print("Hovering over switch", marker.id, "5 seconds before pressing")
        time.sleep(5.0)
        print("pressing switch", marker.id , "in 5 seconds")
        #press switch
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = 0.7071068
        pose_goal.orientation.y = 0.0
        pose_goal.orientation.z = 0.0
        pose_goal.orientation.w = 0.7071068
        pose_goal.position.x = marker.x - 5.0 /100.0
        pose_goal.position.y = marker.y
        pose_goal.position.z = marker.z - 0.065 #6.5 cm
        self.go_to_pose_goal(pose_goal)
        print("reseting to center position")
        #retract arm
        self.go_to_pose_goal(self.center)
        
    def do_objective_2(self):
        """
        2.Push four buttons in a given sequence
        #get list of buttons to press
        #read map file
        #extract button cordinate
        #press button
        # repeat until done
        # according to mechanical drawing, button z = marker z - 5.5 cm ,switch radius  = 30 mm, switch travel  = 23 - 17 = 6 mm in x direction

        """
        file_name = 'map.txt'
        home_directory = os.path.expanduser('~')
        file_location =  home_directory + '/.ros/slam/'
        print("FILE LOCATION:", file_location)
        file_data = open(file_location + file_name)
        markers = []
        i = 0
        for row in file_data:
            markers.append(Marker())
            data = row.split(' ')
            data[len(data) - 1] = data[len(data) - 1].rstrip()
            markers[i].id = int(data[0])
            markers[i].x = float(data[1]) 
            markers[i].y = float(data[2]) 
            markers[i].z = float(data[3]) 
            markers[i].pan = float(data[4]) 
            markers[i].tilt = float(data[5])  
            markers[i].roll = float(data[6])  
            markers[i].variance = float(data[7]) 
            markers[i].numObservations = int(data[8]) 
            markers[i].links= data[9 : None]
            i += 1

        
        switches_to_press = [1]
        if markers is not None:
            for i in range(len(switches_to_press)):
                for j in range(len(markers)):
                    if markers[j].id == switches_to_press[i]:
                        print("switch "+ str(switches_to_press[i]) +" found")
                        
                        self.press_switch(markers[j])
        print("Done Pressing Switches")

    def do_objective_3(self):
        """
        3.Grab the accelerometer
        """
        pass
    def do_objective_4(self):
        """
        4.Attach the accelerometer to the panel

        """
        pass
    def do_objective_5(self):
        """
        5.Open the inspection window cover
        """

        pass
    def do_objective_6(self):
        """
        6.Recognize the marker shown in the inspection window
        """
        pass
    def do_objective_7(self):
        """
        7.Push the button described by the marker shown in the inspection window
        """
        pass
    def do_objective_8(self):
        """
        8.Close the inspection window cover
        """
        pass
    def do_objective_9(self):
        """
        9.Drive the Arm back to its original position : 0, -120, 100, 20, 90, 90
        """

   
        # self.move_group.set_goal_tolerance(0.05)
        # self.move_group.set_pose_target(self.initial_pose)
        # self.plan = self.move_group.go(wait=True)
        # self.move_group.stop()
        # self.move_group.clear_pose_targets()
        # #self.print_current_state()

        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = math.radians(0)
        joint_goal[1] = -math.radians(-120)
        joint_goal[2] = math.radians(100)
        joint_goal[3] = math.radians(20)
        joint_goal[4] = math.radians(90)
        joint_goal[5] = math.radians(-90)

        print(type(joint_goal))
        # self.move_group.go(joint_goal, wait=True)
        # self.move_group.stop()
        # # self.go_to_pose_goal(self.initial_pose)
        # self.print_current_state()
        pass

    def test_gripper(self):
        self.gripper.close()

        self.gripper.semi_close()

        self.gripper.semi_open()

        self.gripper.open() 

        self.gripper.close()
        time.sleep(2.0)



    def do_task_1(self):
        self.do_objective_1()
        #self.do_objective_2()
        pass
    def do_task_2(self):
        self.do_objective_3()
        self.do_objective_4()
        pass
    def do_task_3(self):
        self.do_objective_5()
        self.do_objective_6()
        self.do_objective_7()
        self.do_objective_8()

        pass
    @staticmethod
    def go_to_center():

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = 0.7071068
        pose_goal.orientation.y = 0.0
        pose_goal.orientation.z = 0.0
        pose_goal.orientation.w = 0.7071068
        pose_goal.position.x = 0.31
        pose_goal.position.y = 0.0
        pose_goal.position.z = 0.22
        return pose_goal
    @staticmethod
    def go_to_top_left():
        #mid_left
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = 0.7071068
        pose_goal.orientation.y = 0.0
        pose_goal.orientation.z = 0.0
        pose_goal.orientation.w = 0.7071068
        pose_goal.position.x = 0.25
        pose_goal.position.y = 0.08
        pose_goal.position.z = 0.27
        return pose_goal
    @staticmethod
    def go_to_top_right():
        #mid_right
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = 0.7071068
        pose_goal.orientation.y = 0.0
        pose_goal.orientation.z = 0.0
        pose_goal.orientation.w = 0.7071068
        pose_goal.position.x = 0.25
        pose_goal.position.y = -0.08
        pose_goal.position.z = 0.27
        return pose_goal
    @staticmethod
    def go_to_bot_right():
        #mid_right
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = -0.7071068
        pose_goal.orientation.y = 0.0
        pose_goal.orientation.z = 0.0
        pose_goal.orientation.w = 0.7071068
        pose_goal.position.x = 0.25
        pose_goal.position.y = -0.08
        pose_goal.position.z = 0.27
        return pose_goal
    @staticmethod
    def go_to_bot_left():
        #mid_left
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = -0.7071068
        pose_goal.orientation.y = 0.0
        pose_goal.orientation.z = 0.0
        pose_goal.orientation.w = 0.7071068
        pose_goal.position.x = 0.25
        pose_goal.position.y = 0.08
        pose_goal.position.z = 0.27
        return pose_goal
def main():
    try:
        arm = MoveGroupPythonInteface()
        
        # arm.scan_center_panel()
        arm.print_current_state()

        arm.do_objective_9()

        # #arm.test_gripper()
        # arm.do_task_1()
        # #arm.do_task_2()
        # #arm.do_task_3()
        # arm.do_objective_9()

        # arm.scan_center_panel()

        # gripper1 = Gripper()
        # gripper1.close()

        arm.print_current_state()



        print ("All objectives completed successfully")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
  main()

