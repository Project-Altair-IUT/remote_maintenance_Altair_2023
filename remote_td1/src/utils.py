import rospy
import time
import geometry_msgs
from math import pi, tau, dist, fabs, cos
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import String


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

class Gripper(object):
    def __init__(self):
        # super(Gripper, self).__init__()    
        self.gripper_pub = rospy.Publisher('gripper_command', String, queue_size=10)
        self.state = None
        self.sleep_time = 1
        self.command = "open"

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