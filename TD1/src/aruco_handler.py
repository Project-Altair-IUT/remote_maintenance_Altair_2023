#!/usr/bin/env python

import rospy
from altair_msgs.srv import ArucoService, ArucoServiceResponse

def callback(request):
    return ArucoServiceResponse(request.input, 216)

rospy.init_node('aruco_handler_service_server')
service = rospy.Service('arc', ArucoService, callback)
rospy.spin()