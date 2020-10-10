#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
import math

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
arm_group = moveit_commander.MoveGroupCommander("arm")
gripper_group = moveit_commander.MoveGroupCommander("gripper")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

#get the pose
listener = tf.TransformListener()

while not rospy.is_shutdown():
    try:
        (trans,rot) = listener.lookupTransform('/robot_footprint', '/monster_black', rospy.Time(0))
        print(trans)
        break
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue


#orientation of approach of the end effector
q = tf.transformations.quaternion_from_euler(0,1.57,0.0)

#Object Position
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.x = q[0]
pose_target.orientation.y = q[1]
pose_target.orientation.z = q[2]
pose_target.orientation.w = q[3]
pose_target.position.x = trans[0]
pose_target.position.y = trans[1]
pose_target.position.z = trans[2] 


x_offset_distance = -0.01#-0.1
y_offset_distance = 0.0
z_offset_distance = 0.0#0.03

#Step 1 Approach at a distance

pose_target.position.x = trans[0] + x_offset_distance
pose_target.position.y = trans[1] + y_offset_distance
pose_target.position.z = trans[2] + z_offset_distance


arm_group.set_pose_target(pose_target)
arm_group.set_planning_time(5)
plan_arm = arm_group.plan()
arm_group.go(wait=True)
rospy.sleep(1)