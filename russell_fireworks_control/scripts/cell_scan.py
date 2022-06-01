import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

scan_axes = []
#scan_axes.append([0.0, -0.747000919853573, 1.911135530933791, 0.0, 0.4066617157146788, 0.0])
scan_axes.append([-1.684242728174528, -0.9738937226128358, 2.368411794956305, -0.0017453292519943296, 0.17627825445142728, -0.47647488579445196])
scan_axes.append([-0.3246312408709453, -0.747000919853573, 1.911135530933791, 0.0, 0.4066617157146788, 0.32288591161895097])
scan_axes.append([0.3246312408709453, -0.747000919853573, 1.911135530933791, 0.0, 0.4066617157146788, -0.32288591161895097])
scan_axes.append([1.684242728174528, -0.9738937226128358, 2.368411794956305, 0.0017453292519943296, 0.17627825445142728, 0.47647488579445196])
scan_axes.append([0.0, -1.5708, 1.5708, 0.0, 1.5708, 0.0])

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('russell_fireworks_scan')

robot = moveit_commander.RobotCommander()

group_name = "manipulator"
group = moveit_commander.MoveGroupCommander(group_name)

group.set_max_velocity_scaling_factor(0.05)
group.set_max_acceleration_scaling_factor(0.05)


for i in scan_axes:
    joint_goal = i
    group.go(joint_goal, wait=True)
    group.stop()
