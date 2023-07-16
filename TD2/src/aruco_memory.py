#!/usr/bin/env python3
import pickle
import rospy
import tf2_ros
from geometry_msgs.msg import Transform

from altair_msgs.msg import AltairAruco
from altair_msgs.srv import ArucoService, saverController, saverControllerResponse


from utils import Aruco_Marker, readFile, writeFile, detect_enable


response_wait = 1.0
MEMORY = dict()
markers_to_find = []

pub = rospy.Publisher('/project_altair/aruco_poses', AltairAruco, queue_size=1)

run_saving = False

def callback(request):
    run_saving = request.command
    detect_enable(run_saving)
    markers_to_find = request.markers_to_find

    return saverControllerResponse(True)

def save_transform():
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        for idx in markers_to_find:
            try:
                tf_resultant = tfBuffer.lookup_transform('base_link', 'fiducial_' + str(idx), rospy.Time(), rospy.Duration(response_wait))
                if MEMORY[idx] == None:
                    print(f'{idx} not in memory')
                marker = Aruco_Marker(id = idx)
                marker.pose = tf_resultant.transform
                marker.numObservations += 1
                MEMORY[idx] = marker    #saving the marker object in the dictionary
                print(f'spotted: {idx}') 

            except Exception as e:
                rate.sleep()
                print(f":::->Marker {idx} not found!")
                print(e)
            
        writeFile(MEMORY)        

def main():
    MEMORY = readFile()
    rospy.init_node('project_altair_aruco_memory', anonymous=True)
    service = rospy.Service('saverController', saverController, callback)

    
    ## call save_transform upon conditions
    if run_saving:
        save_transform()

    msg = AltairAruco()
    for key, val in MEMORY.items():
        msg.results_id.append(key)
        msg.results.append(val.pose)
    pub.publish(msg)

        
if __name__ == '__main__':
    main()
