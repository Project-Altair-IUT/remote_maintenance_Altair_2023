#!/usr/bin/env python
import time
import rospy
import tf2_ros
from geometry_msgs.msg import Transform
from utils import Aruco_Marker
from altair_msgs.srv import ArucoService, ArucoServiceResponse

response_wait = 4.0
MEMORY = dict()
Aruco_List = [0 for i in range(15)]
def callback(request):
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(1.0)
    c = 0

    # while not rospy.is_shutdown():
    for idx in range(request.starting_id, request.ending_id+1):
        try:
            trans = tfBuffer.lookup_transform('base_link', 'fiducial_' + str(idx), rospy.Time(), rospy.Duration(response_wait))
            pos = Transform()
            # for some reason the transform needs to be negated
            # except for the z axis?
            # maybe because z axis direction is same for aruco and camera
            # but x and y axis direction is opposite of camera
            detected_aruco = Aruco_Marker()
            detected_aruco.id = idx
            detected_aruco.pose = pos
            pos.translation.x = trans.transform.translation.x
            pos.translation.y = trans.transform.translation.y
            pos.translation.z =  trans.transform.translation.z
            pos.rotation = trans.transform.rotation
            MEMORY[idx] = pos
            Aruco_List[idx] = detected_aruco
            print(c)
            c+=1
            print(pos)
            # time.sleep(0.1)
            
        except Exception as e:
            rate.sleep()
            print(e)
    return ArucoServiceResponse((c == (request.ending_id - request.starting_id +1)), c)

def service_runner():
    rospy.init_node('aruco_handler_server')
    service = rospy.Service('aruco_service', ArucoService, callback)
    rospy.spin()

if __name__ == "__main__":
    service_runner()
