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
group = moveit_commander.MoveGroupCommander("arm")
gripper_group = moveit_commander.MoveGroupCommander("hand")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
positions = [

			#[-1.3,1.2,1.27,1.2,1.2,1.2],
			[0,-0.5,1.5,0,-0.3,0]]#,
			#[0.0,0.0,1.57,0.0,0.0,0.0]]



for pos in positions:
    group_variable_values = group.get_current_joint_values()
    print group_variable_values
    group_variable_values[0] = pos[0]
    group_variable_values[1] = pos[1]
    group_variable_values[2] = pos[2]
    group_variable_values[3] = pos[3]
    group_variable_values[4] = pos[4]
    group_variable_values[5] = pos[5]
    group.set_joint_value_target(group_variable_values)
    plan2 = group.plan()
    group.go(wait=True)
    rospy.sleep(2)
gripper_group.set_named_target("hand_close") 
plan_gripper = gripper_group.go(wait=True)
#rospy.sleep(5)

moveit_commander.roscpp_shutdown()
