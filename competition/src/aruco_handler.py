#!/usr/bin/env python
import pickle
import time
import rospy
import tf2_ros
from geometry_msgs.msg import Transform
from utils import Aruco_Marker, writeFile, readFile, detect_enable

from std_srvs.srv import SetBool
from altair_msgs.srv import ArucoService, ArucoServiceResponse

response_wait = 1.0
MEMORY = dict()


def callback(request):
    detect_enable(True)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    readFile()

    rate = rospy.Rate(1.0)
    markers_found = []

    while (len(markers_found) != len(request.markers_to_find)):
        for idx in request.markers_to_find:
            try:
                    tf_resultant = tfBuffer.lookup_transform('base_link', 'fiducial_' + str(idx), rospy.Time(), rospy.Duration(response_wait))
                    marker = Aruco_Marker(id = idx)
                    marker.pose = tf_resultant
                    marker.numObservations += 1
                    MEMORY[idx] = marker    #saving the marker object in the dictionary
                    print(f'spotted: {idx}') 
                    markers_found.append(idx)

            except Exception as e:
                rate.sleep()
                print(f":::->Marker {idx} not found!")
                print(e)
    
    writeFile()

    detect_enable(False)

    return ArucoServiceResponse((len(markers_found) == len(request.markers_to_find)), markers_found)

def service_runner():
    rospy.init_node('aruco_handler_server')
    service = rospy.Service('aruco_service', ArucoService, callback)
    rospy.spin()

if __name__ == "__main__":
    service_runner()
