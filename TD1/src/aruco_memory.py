#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import Transform
from altair_msgs.msg import AltairAruco

MEMORY = dict()
pub = rospy.Publisher('/project_altair/aruco_poses', AltairAruco, queue_size=1)

def main():
    rospy.init_node('project_altair_aruco_memory', anonymous=True)
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
                pos.translation.x = -trans.transform.translation.x
                pos.translation.y = -trans.transform.translation.y
                pos.translation.z =  trans.transform.translation.z
                pos.rotation = trans.transform.rotation
                MEMORY[idx] = pos
            except Exception as e:
                rate.sleep()
                print(e)
        
        msg = AltairAruco()
        for key, val in MEMORY.items():
            msg.results_id.append(key)
            msg.results.append(val)
        pub.publish(msg)
        
        
if __name__ == '__main__':
    main()
