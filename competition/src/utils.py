#!/usr/bin/env python

import pickle
import rospy
import time
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cos
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import String
from std_srvs.srv import SetBool

from altair_msgs.srv import ArucoService, ArucoServiceResponse, saverController

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


#----------------------------------------------------------------------------------------------------#


class Gripper(object):
    def __init__(self):
        # super(Gripper, self).__init__()    
        self.gripper_pub = rospy.Publisher('gripper_command', String, queue_size=10)
        self.sleep_time = 8
        self.state = None

    def actuate(self, command):
        if command == self.state:
            log_msg = "Gripper already in state : " + command
            rospy.loginfo(log_msg)
            return

        self.state = command
        self.gripper_pub.publish(self.state)

        print(f"Waiting {self.sleep_time} seconds for gripper state change")

        time.sleep(self.sleep_time)

        log_msg = "Gripper state : " + command
        rospy.loginfo(log_msg)

    def close(self):
        self.actuate("close")

    def semi_close(self):
        self.actuate("semi_close")

    def semi_open(self):
        self.actuate("semi_open")

    def open(self):
        self.actuate("open")


#----------------------------------------------------------------------------------------------------#


class Aruco_Marker():
    def __init__(self, id = 0):
        self.id = id
        self.pose = geometry_msgs.msg.Pose() 
        self.numObservations = 0 
        self.links = None
    
    def print_info(self):
        print(f'ID: {self.id}')
        print(self.pose)

    def rapid_update(self):
        pass


#----------------------------------------------------------------------------------------------------#


def writeFile(data):
    try:
        with open('marker_data.pkl', 'wb') as fp:
            pickle.dump(data, fp)
        print('aruco poses saved to file')
        print(data)

    except Exception as f:
        print(f)
    
def readFile():
    try:
        with open('marker_data.pkl', 'rb') as fp:
            data = pickle.load(fp)
        print('aruco poses read from file')
        return data
    except Exception as f:
        print(f)
        return dict()


#----------------------------------------------------------------------------------------------------#


def aruco_saver_caller(command, markers):
    rospy.wait_for_service('saverControllerService')
    try:
        _saverControllerProxy = rospy.ServiceProxy('saverControllerService', saverController)
        request_msg = saverController()
        request_msg.command = command
        request_msg.markers_to_find = markers
        response = _saverControllerProxy(command, markers)

        return response
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


#----------------------------------------------------------------------------------------------------#


def detect_enable(command):
    rospy.wait_for_service('enable_detections')
    try:
        enable_detections = rospy.ServiceProxy('enable_detections', SetBool)
        response = enable_detections(command)
        return response.message
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        

#----------------------------------------------------------------------------------------------------#




