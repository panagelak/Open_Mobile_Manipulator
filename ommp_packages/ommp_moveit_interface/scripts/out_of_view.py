#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import rospy
from std_msgs.msg import Int16
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
arm_group = moveit_commander.MoveGroupCommander("arm")
gripper_group = moveit_commander.MoveGroupCommander("gripper")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
#First go to start position with open gripper
arm_group.set_named_target("out_of_view")
plan_arm = arm_group.plan()
arm_group.go(wait=True)
rospy.sleep(1)
gripper_group.set_named_target("gripper_open") 
plan_gripper = gripper_group.plan()
gripper_group.go(wait=True)
rospy.sleep(1)

moveit_commander.roscpp_shutdown()
