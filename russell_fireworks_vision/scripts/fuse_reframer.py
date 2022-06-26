#!/usr/bin/env python3

import rospy
import tf
import numpy
import math
from tf.transformations import *

frame_names = ["F_15", "F_12", "F_2", "F_0", "F_7", "F_6", "F_11", "F_14", "F_10", "F_16", "F_3", "F_1", "F_5", "F_18", "F_9", "F_13", "F_17", "F_24", "F_22", "F_27", "F_20", "F_26", "F_25", "F_21"]

flame_tip = -1.35

if __name__ == '__main__':
    rospy.init_node('fuse_reframer')

    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()

    fuse_shift = compose_matrix(translate=(0, -0.02, 0))

    rate = rospy.Rate(2.0)
    while not rospy.is_shutdown():
        for f in frame_names:
            try:
                (trans,rot) = listener.lookupTransform('base_link', f, rospy.Time(0))               # look up marker frame
                marker_frame = compose_matrix(translate=trans, angles=euler_from_quaternion(rot))   # create np matrix
                fuse_frame = numpy.matmul(marker_frame, fuse_shift)                                 # Shift the marker frame to line up with  the middle of the fuse
                scale, shear, angles, translate, perspective = decompose_matrix(fuse_frame)         # get the location of the fuse relative to the robot
                yaw = math.atan2(translate[1], translate[0])

                br.sendTransform(translate,
                     tf.transformations.quaternion_from_euler(3.14159, flame_tip, yaw),
                     rospy.Time.now(),
                     f + "_Fuse",
                     "base_link")
            except:
                pass

        rate.sleep()
