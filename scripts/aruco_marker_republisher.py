#!/usr/bin/env python3

import rospy
from rospy import Publisher, Rate, Subscriber
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
from typing import List

NUM_MARKERS = 7;
RATE_VAL = 10;
ARUCO_REPUB = List[Publisher]
ARUCO_SUB = Subscriber

def transform_callback(fiducial_array: FiducialTransformArray):
    global ARUCO_REPUB, NUM_MARKERS
    for transform in fiducial_array.transforms:
        for j in range(NUM_MARKERS):
            if transform.fiducial_id == j:
                new_fiducial_array = FiducialTransformArray()
                new_fiducial_array.header = fiducial_array.header
                new_fiducial_array.image_seq = fiducial_array.image_seq
                new_fiducial_array.transforms = [transform]
                ARUCO_REPUB[j].publish(new_fiducial_array)



def main():
    global RATE_VAL, RATE, ARUCO_REPUB, ARUCO_SUB

    rospy.init_node('aruco_marker_republisher')
    RATE = Rate(hz=RATE_VAL)
    ARUCO_REPUB = [Publisher('Aruco_Marker_'+ str(_), FiducialTransformArray, queue_size=1) for _ in range(NUM_MARKERS)]
    ARUCO_SUB = Subscriber('/fiducial_transforms', FiducialTransformArray, transform_callback)

    rospy.spin()

if __name__ == '__main__':
    main()

