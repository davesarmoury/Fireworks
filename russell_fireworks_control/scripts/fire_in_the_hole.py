#!/usr/bin/env python3

import rospy
import moveit_commander
import moveit_msgs.msg
from tf.transformations import quaternion_from_euler
import yaml
import sys
import tf
import geometry_msgs
from std_msgs.msg import Bool

home = [0.0, -1.160643952576229, 2.558652683423687, 0.0, 0.2984513020910304, 0.0]

frame_naming = ["F_", "_Fuse"]

with open("../data/show.yaml", 'r') as stream:
    show = yaml.safe_load(stream)

def fire():
    global fire_pub
    fire_pub.publish(True)
    rospy.sleep(1.4)
    fire_pub.publish(False)
    rospy.sleep(0.1)

def cart_goal(pos, quat=False):
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = pos[0]
    pose_goal.position.y = pos[1]
    pose_goal.position.z = pos[2]

    if quat:
        q = [pos[3], pos[4], pos[5], pos[6]]
    else:
        q = quaternion_from_euler(pos[3], pos[4], pos[5])

    pose_goal.orientation.x = q[0]
    pose_goal.orientation.y = q[1]
    pose_goal.orientation.z = q[2]
    pose_goal.orientation.w = q[3]

    return pose_goal

def cart_joint_move(group, pos, quat=False):
    pose_goal = cart_goal(pos, quat)

    group.set_pose_target(pose_goal)

    plan = group.go(wait=True)
    group.stop()

    group.clear_pose_targets()

def cart_lin_move(group, pos, continuous=True):
    waypoints = []
    waypoints.append(cart_goal(pos))

    if continuous:
        cart_step = 0.01
    else:
        cart_step = 0.1

    replans = 10

    while replans > 0:
        (plan, fraction) = group.compute_cartesian_path(waypoints, cart_step, 0.0)
        if fraction > 0.99:
            if continuous:
                group.execute(plan, wait=True)
            else:
                for p in plan.joint_trajectory.points:
                    group.go(p.positions, wait=True)
                    group.stop()
            group.stop()
            break
        replans = replans - 1
    else:
        rospy.logwarn("Replanning failed")

def main():
    global fire_pub
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('russell_fireworks')

    fire_pub = rospy.Publisher('/fire', Bool, queue_size=1)

    listener = tf.TransformListener()

    robot = moveit_commander.RobotCommander()

    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)

    group.set_max_velocity_scaling_factor(0.1)
    group.set_max_acceleration_scaling_factor(0.1)

    group.go(home, wait=True)
    group.stop()

    for fw in show["show_order"]:
        fname = frame_naming[0] + str(fw["number"]) + frame_naming[1]
        rospy.loginfo(fname)
        listener.waitForTransform("base_link", fname, rospy.Time(), rospy.Duration(1.0))
        (trans,rot) = listener.lookupTransform('/base_link', fname, rospy.Time(0))
        rospy.loginfo(rot)
        pose = [trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3]]
        cart_joint_move(group, pose, True)
        fire()

        if fw["home"]:
            group.go(home, wait=True)
            group.stop()

        rospy.sleep(fw["delay"])

    group.go(home, wait=True)
    group.stop()

main()
