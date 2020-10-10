
#! /bin/bash


x-terminal-emulator -e roslaunch ommp_bringup sim_bringup.launch world:=simple 2>/dev/null &&

sleep 8 &&

x-terminal-emulator -e roslaunch ommp_moveit_interface moveit.launch 2>/dev/null &&

sleep 2 &&

x-terminal-emulator -e rosrun ommp_moveit_interface set_start_pos.py 2>/dev/null &&

sleep 1 &&

x-terminal-emulator -e rostopic pub -1 /kinect_controller/command std_msgs/Float64 "data: 0.0" 2>/dev/null &&

sleep 1 &&

x-terminal-emulator -e roslaunch ommp_navigation navigation_main.launch map:=sim_map 2>/dev/null &&

sleep 6 &&

x-terminal-emulator -e roslaunch ommp_viz rviz.launch config:=navigation 2>/dev/null &&

sleep 4 &&


x-terminal-emulator -e rosrun ommp_navigation patrol.py 2>/dev/null &
