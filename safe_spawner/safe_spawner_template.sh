
#! /bin/bash

echo ' '
read -p 'Would you like to clear the previous map database? (y/n): ' ansinput

if [ “$ansinput” = “y” ]
then
 printf '\n Map deleted \n'
 rm -f ~/.ros/rtabmap.db

elif [ “$ansinput” = “n” ]
then
 printf '\n Map kept \n'

else
 echo 'Warning: Not an acceptable option. Choose (y/n).
         '
fi

echo ' '

read -p 'Enter target world destination or d for default: ' input_choice

if [ “$input_choice” = “d” ]
then
 x-terminal-emulator -x roslaunch slam_project world.launch world_file:=/home/makemedie/catkin_ws/src/slam_project/worlds/designed_world.world 2>/dev/null &

else
 x-terminal-emulator -x roslaunch slam_project world.launch world_file:=$input_choice 2>/dev/null &
fi

sleep 3 &&

x-terminal-emulator -x roslaunch slam_project teleop.launch 2>/dev/null &

sleep 3 &&

echo ' '
read -p 'Press any key to continue to mapping... ' -n1 -s

x-terminal-emulator -x roslaunch slam_project mapping.launch simulation:=true 2>/dev/null &
sleep 3 &&
x-terminal-emulator -e roslaunch slam_project rviz.launch 2>/dev/null

echo ' '
echo 'Script Completed'
echo ' '
