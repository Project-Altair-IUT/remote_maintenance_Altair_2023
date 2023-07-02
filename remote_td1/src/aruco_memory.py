#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import tf
from vision_msgs.msg import Detection2DArray
from altair_msgs.msg import AltairAruco

MEMORY = dict()
pub = rospy.Publisher('/project_altair/aruco_poses', AltairAruco, queue_size=1)

def convert_from_camera_to_base(pose):
    listener = tf.TransformListener()
    listener.waitForTransform('/base', '/camera_link', rospy.Time(), rospy.Duration(5.0))
    t = listener.getLatestCommonTime('/base', '/camera_link')
    p1 = PoseStamped()
    p1.header.frame_id = 'camera_link'
    return listener.transformPose('/base', p1).pose


def callback(data):
    for d in data.detections:
        r = d.results[0]
        if r.score > .90:
            r.pose.pose = convert_from_camera_to_base(r.pose.pose)
            MEMORY[r.id] = r
            
    pub.publish([val for _, val in MEMORY.items()])

def main():
    rospy.init_node('project_altair_aruco_memory', anonymous=True)
    rospy.Subscriber('/fiducial_transforms', Detection2DArray, callback)
    rospy.spin()

if __name__ == '__main__':
    main()
