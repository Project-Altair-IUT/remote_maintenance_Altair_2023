#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import Transform
from altair_msgs.srv import ArucoService, ArucoServiceResponse

MEMORY = dict()

def callback(request):
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        for idx in range(1, 15):
            try:
                trans = tfBuffer.lookup_transform('base', 'fiducial_' + str(idx), rospy.Time())
                pos = Transform()
                # for some reason the transform needs to be negated
                # except for the z axis?
                # maybe because z axis direction is same for aruco and camera
                # but x and y axis direction is opposite of camera
                pos.translation.x = -trans.transform.translation.x
                pos.translation.y = -trans.transform.translation.y
                pos.translation.z =  trans.transform.translation.z
                pos.rotation = trans.transform.rotation
                MEMORY[idx] = pos
            except Exception as e:
                rate.sleep()
                print(e)
    return ArucoServiceResponse(True, request.input)

def service_runner():
    rospy.init_node('aruco_handler_server')
    service = rospy.Service('aruco_service', ArucoService, callback)
    rospy.spin()

if __name__ == "__main__":
    service_runner()
