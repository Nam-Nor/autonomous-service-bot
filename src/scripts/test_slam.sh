#!/bin/sh
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/map/myworld.world " &
sleep 5
xterm  -e  " roslaunch turtlebot_teleop keyboard_teleop.launch" & 
sleep 5
xterm  -e  " rosrun gmapping slam_gmapping" &
sleep 5
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch"