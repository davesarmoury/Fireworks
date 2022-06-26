#!/usr/bin/env python3
from fiducial_msgs.msg import FiducialTransformArray
import rospy
import tf
from tf.transformations import *
import numpy
from threading import Lock

tolerance = 0.1
frames = {}
dict_lock = Lock()

def callback(msg):
    global frames
    temp_frames = {}

    if len(msg.transforms) > 0:
        (trans,rot) = listener.lookupTransform('/base_link', '/zed2i_left_camera_optical_frame', rospy.Time(0))  # Bad form to hard-code, but I feel lazy today
        base_cam_mtx = compose_matrix(translate=trans, angles=euler_from_quaternion(rot))

        for t in msg.transforms:
            if t.object_error < tolerance and t.object_error < frames.get(t.fiducial_id, [99999])[0]:
                mtf = t.transform
                mtrans = (mtf.translation.x, mtf.translation.y, mtf.translation.z)
                mrot = (mtf.rotation.x, mtf.rotation.y, mtf.rotation.z, mtf.rotation.w)
                cam_marker_mtx = compose_matrix(translate=mtrans, angles=euler_from_quaternion(mrot))
                base_marker_mtx = numpy.matmul(base_cam_mtx, cam_marker_mtx)
                temp_frames[t.fiducial_id] = [t.object_error, base_marker_mtx]

        dict_lock.acquire()
        for i, f in temp_frames.items():
            frames[i] = f
        dict_lock.release()

def listener():
    global listener, frames
    rospy.init_node('fiducial_updater', anonymous=True)

    listener = tf.TransformListener()

    rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, callback)

    br = tf.TransformBroadcaster()

    rate = rospy.Rate(2.0)

    while not rospy.is_shutdown():
        dict_lock.acquire()
        for i, f in frames.items():
            scale, shear, angles, translate, perspective = decompose_matrix(f[1])
            br.sendTransform(translate,
                     tf.transformations.quaternion_from_euler(angles[0], angles[1], angles[2]),
                     rospy.Time.now(),
                     "F_" + str(i),
                     "base_link")

        print("TOTAL RECORDED: " + str(len(frames)))
        dict_lock.release()

        rate.sleep()

if __name__ == '__main__':
    listener()
