#! /bin/bash

x-terminal-emulator -e roslaunch ommp_bringup sim_bringup.launch world:=simple headless:=true 2>/dev/null &&

sleep 8 &&

x-terminal-emulator -e roslaunch ommp_moveit_interface moveit.launch 2>/dev/null &&

sleep 3 &&

x-terminal-emulator -e rosrun ommp_moveit_interface set_start_pos.py 2>/dev/null &&

sleep 1 &&

x-terminal-emulator -e roslaunch ommp_navigation gmapping.launch 2>/dev/null &&

sleep 3 &&

x-terminal-emulator -e roslaunch ommp_navigation move_base.launch 2>/dev/null &&

sleep 3 &&

x-terminal-emulator -e roslaunch exploration_server exploration.launch 2>/dev/null &&

sleep 5 &&

x-terminal-emulator -e roslaunch ommp_viz rviz.launch config:=frontier_exploration 2>/dev/null &