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

#First go to start position with open gripper
arm_group.set_named_target("out_of_view")
plan_arm = arm_group.plan()
arm_group.go(wait=True)
rospy.sleep(5)

while not rospy.is_shutdown():
    try:
        (trans,rot) = listener.lookupTransform('/robot_footprint', '/monster_black', rospy.Time(0))
        print(trans)
        break
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue


gripper_group.set_named_target("gripper_open") 
plan_gripper = gripper_group.plan()
gripper_group.go(wait=True)
rospy.sleep(1)

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


x_offset_distance = -0.07#-0.1
y_offset_distance = 0.0
z_offset_distance = 0.0#0.03

x_offset_approach = -0.02
y_offset_approach = 0.0
z_offset_approach = 0.0#0.03


x_offset_lift = -0.02
y_offset_lift = 0.0
z_offset_lift = 0.04
#Step 1 Approach at a distance

pose_target.position.x = trans[0] + x_offset_distance
pose_target.position.y = trans[1] + y_offset_distance
pose_target.position.z = trans[2] + z_offset_distance


arm_group.set_pose_target(pose_target)
plan_arm = arm_group.plan()
arm_group.go(wait=True)
rospy.sleep(1)

#Step 2 Close in on the object and close the gripper

pose_target.position.x = trans[0] + x_offset_approach
pose_target.position.y = trans[1] + y_offset_approach
pose_target.position.z = trans[2] + z_offset_approach

arm_group.set_pose_target(pose_target)
plan_arm = arm_group.plan()
arm_group.go(wait=True)
rospy.sleep(1)

gripper_group.set_named_target("gripper_close") 
plan_gripper = gripper_group.go(wait=True)
rospy.sleep(1)

# Step 3 Lift the object off the ground

pose_target.position.x = trans[0] + x_offset_lift
pose_target.position.y = trans[1] + y_offset_lift
pose_target.position.z = trans[2] + z_offset_lift

arm_group.set_pose_target(pose_target)
plan_arm = arm_group.plan()
arm_group.go(wait=True)
rospy.sleep(1)


# Step 4 Go the object to the desired position

group_variable_values = arm_group.get_current_joint_values()
group_variable_values[0] = 1.57
group_variable_values[1] = 0.0
group_variable_values[2] = 1.57
group_variable_values[3] = 0.0
group_variable_values[4] = 0.0
group_variable_values[5] = 0.0
arm_group.set_joint_value_target(group_variable_values)

plan_arm = arm_group.plan()
arm_group.go(wait=True)
rospy.sleep(1)

gripper_group.set_named_target("gripper_open") 
plan_gripper = gripper_group.go(wait=True)
rospy.sleep(1)

#Return at start
arm_group.set_named_target("out_of_view")
plan_arm = arm_group.plan()
arm_group.go(wait=True)
rospy.sleep(1)


moveit_commander.roscpp_shutdown()
