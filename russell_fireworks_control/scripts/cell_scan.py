#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler

home = [0.0, -1.5708, 1.5708, 0.0, 1.5708, 0.0]

cart_poses = []
cart_poses.append([0.027, 0.560, 0.231, -3.140, -1.066, 1.176])
cart_poses.append([0.767, 0.267, 0.203, 3.139, -1.364, 0.820])
cart_poses.append([0.767, -0.267, 0.203, 3.139, -1.364, -0.820])
cart_poses.append([0.027, -0.560, 0.231, -3.140, -1.066, -1.176])

def cart_goal(pos):
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = pos[0]
    pose_goal.position.y = pos[1]
    pose_goal.position.z = pos[2]

    q = quaternion_from_euler(pos[3], pos[4], pos[5])
    pose_goal.orientation.x = q[0]
    pose_goal.orientation.y = q[1]
    pose_goal.orientation.z = q[2]
    pose_goal.orientation.w = q[3]

    return pose_goal

def cart_joint_move(group, pos):
    pose_goal = cart_goal(pos)

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
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('russell_fireworks_scan')

    robot = moveit_commander.RobotCommander()

    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)

    group.set_max_velocity_scaling_factor(0.1)
    group.set_max_acceleration_scaling_factor(0.1)

    group.go(home, wait=True)
    group.stop()

    cart_joint_move(group, cart_poses[1])
    cart_lin_move(group, cart_poses[0])

    group.set_max_velocity_scaling_factor(0.5)
    group.set_max_acceleration_scaling_factor(0.5)

    cart_lin_move(group, cart_poses[1], False)
    cart_lin_move(group, cart_poses[2], False)
    cart_lin_move(group, cart_poses[3], False)

    group.set_max_velocity_scaling_factor(0.1)
    group.set_max_acceleration_scaling_factor(0.1)

    cart_lin_move(group, cart_poses[2])

    group.go(home, wait=True)
    group.stop()

main()
