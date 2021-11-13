#!/usr/local/bin/python3

import rospy
from rospy import Publisher, Rate, Subscriber
from fiducial_msgs.msg import _FiducialTransform, FiducialTransformArray
from typing import List

NUM_MARKERS = 3;
RATE_VAL = 10;
ARUCO_REPUB = List[Publisher]
ARUCO_SUB = Subscriber

def transform_callback(fiducial_array: FiducialTransformArray):
    global ARUCO_REPUB, NUM_MARKERS
    for i in fiducial_array.length:
        for j in range(NUM_MARKERS):
            if fiducial_array.transforms[i].fiducial_id == j:
                ARUCO_REPUB[j].publish(fiducial_array.transforms[i])



def main():
    global RATE_VAL, RATE, ARUCO_REPUB, ARUCO_SUB

    rospy.init_node('aruco_marker_republisher')
    RATE = Rate(hz=RATE_VAL)
    ARUCO_REPUB = [Publisher('Aruco_Marker_'+ _, _FiducialTransform) for _ in range(NUM_MARKERS)]
    ARUCO_SUB = Subscriber('/fiducial_transforms', FiducialTransformArray, transform_callback)

    rospy.spin()

if __name__ == '__main__':
    main()

