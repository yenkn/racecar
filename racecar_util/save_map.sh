#!/bin/zsh

rosrun map_server map_saver -f "$ROS_MYPATH/src/racecar_navigation/maps/mymap"
# image2gridmap -i "$ROS_MYPATH/src/racecar_navigation/maps/mymap.pgm" -o "$ROS_MYPATH/src/racecar_navigation/maps/mymap.gridmap" -w --res 0.05
