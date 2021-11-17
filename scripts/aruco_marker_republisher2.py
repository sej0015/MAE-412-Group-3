#!/usr/bin/env python3

import rospy
from rospy import Publisher, Rate, Subscriber
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import PoseArray
from typing import List

RATE_VAL = 10;
ARUCO_REPUB = List[Publisher]
ARUCO_SUB = Subscriber

def transform_callback(fiducial_array: FiducialTransformArray):
    global ARUCO_REPUB
        
    pose_msgs = PoseArray()
    pose_msgs.header = fiducial_array.header
    for i in range(len(fiducial_array.transforms)):
        pose_msgs.poses(i).position.x = fiducial_array.transforms(i).transform.translation.x
        pose_msgs.poses(i).position.y = fiducial_array.transforms(i).transform.translation.y
        pose_msgs.poses(i).position.z = fiducial_array.transforms(i).transform.translation.z
        pose_msgs.poses(i).orientation.w = fiducial_array.transforms(i).id
        ARUCO_REPUB.publish(pose_msgs)


def main():
    global RATE_VAL, RATE, ARUCO_REPUB, ARUCO_SUB

    rospy.init_node('aruco_marker_republisher')
    RATE = Rate(hz=RATE_VAL)
    ARUCO_REPUB = Publisher('/marker_points', PoseArray, queue_size=1)
    ARUCO_SUB = Subscriber('/fiducial_transforms', FiducialTransformArray, transform_callback)

    rospy.spin()

if __name__ == '__main__':
    main()

