
#! /bin/bash


x-terminal-emulator -e roslaunch ommp_bringup real_bringup.launch 2>/dev/null &&

sleep 3 &&

x-terminal-emulator -e roslaunch ommp_moveit_interface moveit_real.launch 2>/dev/null &&

sleep 2 &&

x-terminal-emulator -e rosrun ommp_moveit_interface set_start_pos.py 2>/dev/null &&

sleep 1 &&

x-terminal-emulator -e roslaunch ommp_control teleop.launch 2>/dev/null &&

sleep 1 &&

x-terminal-emulator -e roslaunch ommp_viz rviz.launch config:=pseudo_real 2>/dev/null &



