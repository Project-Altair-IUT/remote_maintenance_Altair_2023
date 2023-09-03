#!/usr/bin/env python3
import pickle
import rospy
import time
import tf2_ros
from geometry_msgs.msg import Transform

from altair_msgs.msg import AltairAruco
from altair_msgs.srv import ArucoService, saverController, saverControllerResponse

from utils import Aruco_Marker, readFile, writeFile, detect_enable


response_wait = 1.0
MEMORY = dict()
markers_to_find = []
run_saving = False

def callback(request):
    detect_enable(request.command)
    global run_saving, markers_to_find
    run_saving = request.command
    markers_to_find = request.markers_to_find
    
    return saverControllerResponse(True)

def save_transform(tfBuffer):
    print('enabled save transform')
    print(f'markers looking for: {markers_to_find}')
    global MEMORY
    for idx in markers_to_find:
            print(f'searching for {idx}')
            try:
                tf_resultant = tfBuffer.lookup_transform('base_link', 'fiducial_' + str(idx), rospy.Time(), rospy.Duration(response_wait))
                
                marker = Aruco_Marker(id = idx)
                marker.pose = tf_resultant.transform
                marker.numObservations += 1
                MEMORY[idx] = marker    #saving the marker object in the dictionary
                print(f'spotted: {idx}') 
                # print(tf_resultant.transform)

            except Exception as e:
                print(e)

    writeFile(MEMORY)

def main():
    detect_enable(False)

    global MEMORY
    MEMORY = readFile()

    rospy.init_node('project_altair_aruco_memory', anonymous=True)
    pub = rospy.Publisher('/project_altair/aruco_poses', AltairAruco, queue_size=1)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(2)   

    try:
        service = rospy.Service('saverControllerService', saverController, callback)
        print('service saver controller started successfully')
    except Exception as e:
        print(e)

    
    while not rospy.is_shutdown():    
        ## call save_transform upon conditions
        if run_saving:
            save_transform(tfBuffer)

        try: 
            msg = AltairAruco()
            for key, val in MEMORY.items():
                msg.results_id.append(key)
                msg.results.append(val.pose)
            pub.publish(msg)
        except Exception as e:
            print(e)

        rate.sleep()            
        
if __name__ == '__main__':
    main()
