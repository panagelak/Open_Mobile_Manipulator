
#! /bin/bash


x-terminal-emulator -e roslaunch ommp_bringup sim_bringup.launch world:=object_recognition1 2>/dev/null &&

sleep 10 &&

x-terminal-emulator -e roslaunch ommp_moveit_interface moveit.launch 2>/dev/null &&

sleep 2 &&

x-terminal-emulator -e rosrun ommp_moveit_interface out_of_view.py 2>/dev/null &&

sleep 1 &&

x-terminal-emulator -e rostopic pub -1 /kinect_controller/command std_msgs/Float64 "data: 0.64" 2>/dev/null &&

sleep 1 &&

x-terminal-emulator -e roslaunch sensor_stick object_recognition.launch 2>/dev/null &&

sleep 8 &&

x-terminal-emulator -e roslaunch ommp_viz rviz.launch config:=object_recognition1 2>/dev/null &&

sleep 8 &&

x-terminal-emulator -e roslaunch ommp_moveit_interface pick_place.launch 2>/dev/null &&

sleep 8 &&

x-terminal-emulator -e rosrun ommp_moveit_interface pick_place_client.py 2>/dev/null &






