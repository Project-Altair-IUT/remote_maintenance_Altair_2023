#!/usr/bin/env python3
import pickle
import rospy
import tf2_ros
from geometry_msgs.msg import Transform

from altair_msgs.msg import AltairAruco

from utils import Aruco_Marker

response_wait = 3.0
MEMORY = dict()
pub = rospy.Publisher('/project_altair/aruco_poses', AltairAruco, queue_size=1)

def writeFile():
    try:
        with open('marker_data.pkl', 'wb') as fp:
            pickle.dump(MEMORY, fp)
            print('aruco poses saved to file')

    except Exception as f:
        print(f)
    
def readFile():
    try:
        with open('marker_data.pkl', 'rb') as fp:
            MEMORY = pickle.load(fp)
            print('aruco poses read from file')

    except Exception as f:
        print(f)
    

def main():
    try:
        readFile()
    except Exception as f:
        print(f)
    rospy.init_node('project_altair_aruco_memory', anonymous=True)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        for idx in range(1, 15):
            try:
                tf_resultant = tfBuffer.lookup_transform('base_link', 'fiducial_' + str(idx), rospy.Time(), rospy.Duration(response_wait))
                marker = Aruco_Marker(id = idx)
                marker.pose = tf_resultant
                marker.numObservations += 1
                MEMORY[idx] = marker

            except Exception as e:
                rate.sleep()
                print(e)
        
        msg = AltairAruco()
        for key, val in MEMORY.items():
            msg.results_id.append(key)
            msg.results.append(val.pose)
        pub.publish(msg)
    
    writeFile()
                
if __name__ == '__main__':
    main()
