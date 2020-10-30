#!/usr/bin/env python
import time
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *

# For teleoperation
import sys, select, termios, tty 
import numpy as np

from rain_unity.msg import rain_system  as RainMsg 
from rain_unity.msg import Human_orion  as LeapMsg
from geometry_msgs.msg import TwistStamped


import math
import copy

###############################################################
###############################################################
num_stack_palm = 30 # Stack for Delta of Euler Angle of Hand
publish_rate = 125
time_to_reach = 0.008
scale_factor_orientation = 100*5# 100
scale_factor_position = 100*5 #100
incoming_command_timeout = 0.1 # (sec)
###############################################################
###############################################################

global LeapMsg_from_unity, teleoperation_mode
teleoperation_mode = None 
LeapMsg_from_unity = None

global del_Euler_stack, del_Euler_filtered
global time_pre_cbLEAP, palm_frame_pre

del_Euler_stack = None
del_Euler_filtered = None
time_pre_cbLEAP = None
palm_frame_pre = None

def callback_Mode(status):
    global teleoperation_mode
    teleoperation_mode = copy.deepcopy(status.teleoperation_mode)

def callback_Leap(status): 
    global LeapMsg_from_unity, time_pre_cbLEAP
    LeapMsg_from_unity = copy.deepcopy(status)
    time_pre_cbLEAP = rospy.get_rostime()

    

def get_Euler(X_current, X_ref): # Get Euler Angles in Global Frame
    gimbal_lock_flag = 0
    
    # Calculate r p y
    X_current_X_ref_inv = np.matmul(X_current, np.linalg.inv(X_ref))


    # Roll => Output range: -2pi to 2pi
    m_cos_p_sin_r = X_current_X_ref_inv[1,2]
    cos_p_cos_r = X_current_X_ref_inv[2,2]
    if m_cos_p_sin_r == 0 and cos_p_cos_r == 0: # Gimbal lock case
        print("Gimbal lock happens at Roll! \n")
        gimbal_lock_flag = 1
    else:
        roll = math.atan2(-m_cos_p_sin_r,cos_p_cos_r)

       
    # Pitch => -pi to pi. because it is not easy to get minus cos_p) 
    sin_p = X_current_X_ref_inv[0,2]
    sin_r = math.sin(roll)
    cos_r = math.cos(roll)
    cos_p = cos_p_cos_r/cos_r
    # cos_p = -m_cos_p_sin_r/sin_r
    # cos_p = math.sqrt(m_cos_p_sin_r**2 + cos_p_cos_r**2)
    pitch = math.atan2(sin_p,cos_p)
    
    # Yaw => -2pi to 2pi
    m_cos_p_sin_y = X_current_X_ref_inv[0,1]
    cos_p_cos_y = X_current_X_ref_inv[0,0]
    if m_cos_p_sin_y == 0 and cos_p_cos_y == 0: # Gimbal lock case
        print("Gimbal lock happens at Yaw! \n")
        gimbal_lock_flag = 1
    else:
        yaw = math.atan2(-m_cos_p_sin_y,cos_p_cos_y)

    Euler = np.array([roll, pitch, yaw])
    return Euler

def leap_to_twist_callback(Coordinate):
    global time_pre_cbLEAP, palm_frame_pre, del_Euler_stack # For computing del_Euler     
    global twist_pub_, LeapMsg_from_unity  
    
    rate = rospy.Rate(publish_rate)
    time_pre_cbLEAP = rospy.get_rostime() # Initialise

    while not rospy.is_shutdown():
        time_now = rospy.get_rostime()
        latency = time_now.to_sec() - time_pre_cbLEAP.to_sec()
        if LeapMsg_from_unity is not None and teleoperation_mode == "MODE_1" and latency < incoming_command_timeout:
            
            ##### Right Hand
            if (LeapMsg_from_unity.right_hand.is_present is True):

                twist = TwistStamped()

                # Coordination transformation from Unity to ROS
                palm_velocity_from_unity = copy.deepcopy(LeapMsg_from_unity.right_hand.palm_velocity)
                palm_normal_from_unity = copy.deepcopy(LeapMsg_from_unity.right_hand.palm_normal)
                palm_direction_from_unity = copy.deepcopy(LeapMsg_from_unity.right_hand.palm_direction)                 

                if Coordinate == "Unity":
                    # // e.g. ROS (x,y,z) = Unity (z, -x, y)
                    palm_velocity = (palm_velocity_from_unity[2], -palm_velocity_from_unity[0], palm_velocity_from_unity[1])
                    palm_normal = np.array([palm_normal_from_unity.z,-palm_normal_from_unity.x, palm_normal_from_unity.y])
                    palm_direction = np.array([palm_direction_from_unity.z,-palm_direction_from_unity.x, palm_direction_from_unity.y])
                

                elif Coordinate == "ROS":
                    # Assuming that the information has been already transformed. 
                    palm_velocity = (palm_velocity_from_ros[0], palm_velocity_from_ros[1], palm_velocity_from_ros[2])
                    palm_normal = np.array([palm_normal_from_ros.x,palm_normal_from_ros.y, palm_normal_from_ros.z])
                    palm_direction = np.array([palm_direction_from_ros.x,palm_direction_from_ros.y, palm_direction_from_ros.z])
                
                elif Coordinate == "demo1":
                    # // For Bare-Hand Demo // e.g. ROS (x,y,z) = Unity (-x, -z, y)
                    palm_velocity = (-palm_velocity_from_unity[0], -palm_velocity_from_unity[2], palm_velocity_from_unity[1])
                    palm_normal = np.array([-palm_normal_from_unity.x,-palm_normal_from_unity.z, palm_normal_from_unity.y])
                    palm_direction = np.array([-palm_direction_from_unity.x,-palm_direction_from_unity.z, palm_direction_from_unity.y])

                # Get Delta_Position
                twist.twist.linear.x = palm_velocity[0]*time_to_reach*scale_factor_position
                twist.twist.linear.y = palm_velocity[1]*time_to_reach*scale_factor_position
                twist.twist.linear.z = palm_velocity[2]*time_to_reach*scale_factor_position                    

                # Get Delta_Euler
                dir_cross_nor_vec_ = np.cross(palm_direction,palm_normal)
                palm_frame = np.asmatrix(np.transpose([palm_normal, palm_direction, dir_cross_nor_vec_]))

                if palm_frame_pre is not None: 
                    
                    del_Euler_ = get_Euler(palm_frame,palm_frame_pre)

                    if del_Euler_stack is None:
                        del_Euler_stack = [del_Euler_]*num_stack_palm
                    else:
                        del_Euler_stack.pop(0) 
                        del_Euler_stack.append(del_Euler_)
                    del_Euler_filtered = np.mean(del_Euler_stack, axis = 0)
                
                else:
                    del_Euler_filtered = np.array([0, 0, 0])

                palm_frame_pre = copy.deepcopy(palm_frame)
                
                twist.twist.angular.x = del_Euler_filtered[0]*scale_factor_orientation
                twist.twist.angular.y = del_Euler_filtered[1]*scale_factor_orientation
                twist.twist.angular.z = del_Euler_filtered[2]*scale_factor_orientation

                
                twist.header.stamp = rospy.get_rostime()
                twist_pub_.publish(twist)
                rate.sleep()






def main(Arg):
    global twist_pub_

    if (Arg=='Unity') or (Arg=='ROS') or (Arg=='demo1'):
        try:
            rospy.init_node("leapmotion_to_twist", anonymous=True, disable_signals=True)
                
            twist_pub_ = rospy.Publisher('/jog_server/delta_jog_cmds',TwistStamped,queue_size=1)

            rospy.Subscriber("/rain/status",RainMsg, callback_Mode, queue_size=1)     
            rospy.Subscriber("/rain/leap_motion",LeapMsg, callback_Leap, queue_size=1)

            print("Taking Leap Motion in ", Arg, " Coordinate")
            leap_to_twist_callback(Arg)
            # leap_to_twist()
        
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise
    else:
        rospy.signal_shutdown("Wrong Argument: It should be either 'ROS' or 'Unity'[default]")


if __name__ == '__main__':  
    if len(sys.argv) < 2:        
        main('Unity')
    else:   
        main(sys.argv[1])
