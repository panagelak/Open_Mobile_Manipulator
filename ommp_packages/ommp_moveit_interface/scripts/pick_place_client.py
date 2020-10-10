#! /usr/bin/env python
import sys
import copy
import rospy
from geometry_msgs.msg import Pose
import tf
import math
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from ommp_moveit_interface.srv import *
#from rospy_message_converter import message_converter
import yaml
import random
from math import pi

rospy.init_node('pick_place_client', anonymous=True)

dist_pose = Pose()
pick_pose = Pose()
lift_pose = Pose()

q = tf.transformations.quaternion_from_euler(0,1.57,0.0)


x_offset_dist = -0.07#-0.1
y_offset_dist = 0.0
z_offset_dist = 0.0#0.03

x_offset_pick = -0.02
y_offset_pick = 0.0
z_offset_pick = 0.0#0.03


x_offset_lift = -0.02
y_offset_lift = 0.0
z_offset_lift = 0.1

#get the pose
listener = tf.TransformListener()

while not rospy.is_shutdown():
    try:
        (trans,rot) = listener.lookupTransform('/robot_footprint', '/box_red', rospy.Time(0))
        print(trans)
        break
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue

dist_pose.orientation.x = q[0]
dist_pose.orientation.y = q[1]
dist_pose.orientation.z = q[2]
dist_pose.orientation.w = q[3]
dist_pose.position.x = trans[0] + x_offset_dist
dist_pose.position.y = trans[1] + y_offset_dist
dist_pose.position.z = trans[2] + z_offset_dist

pick_pose.orientation.x = q[0]
pick_pose.orientation.y = q[1]
pick_pose.orientation.z = q[2]
pick_pose.orientation.w = q[3]
pick_pose.position.x = trans[0] + x_offset_pick
pick_pose.position.y = trans[1] + y_offset_pick
pick_pose.position.z = trans[2] + z_offset_pick

lift_pose.orientation.x = q[0]
lift_pose.orientation.y = q[1]
lift_pose.orientation.z = q[2]
lift_pose.orientation.w = q[3]
lift_pose.position.x = trans[0] + x_offset_lift
lift_pose.position.y = trans[1] + y_offset_lift
lift_pose.position.z = trans[2] +z_offset_lift

# Wait for 'pick_place_routine' service to come up
rospy.wait_for_service('pick_place_routine')
#'''
try:
    pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

    # TODO: Insert your message variables to be sent as a service request
    resp = pick_place_routine(dist_pose, pick_pose, lift_pose)

    print ("Response: ",resp.success)

except rospy.ServiceException, e:
    print "Service call failed: %s"%e