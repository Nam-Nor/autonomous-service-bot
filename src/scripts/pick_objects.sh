#!/bin/sh
xterm -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/autonomous-service-bot/src/map/myworld.world" &
sleep 5
xterm -e  " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/autonomous-service-bot/src/map/map.yaml" & 
sleep 5
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch" & 
sleep 5
xterm -e " rosrun pick_objects pick_objects "