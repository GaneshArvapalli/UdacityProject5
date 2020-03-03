#!/bin/sh
xterm  -e  " source /opt/ros/kinetic/setup.bash; roscore" & 
sleep 5
xterm  -e  " source /home/workspace/UdacityProject5/catkin_ws/devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch" & 
sleep 5
xterm  -e  " source /home/workspace/UdacityProject5/catkin_ws/devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch" & 
sleep 5
xterm  -e  " source /home/workspace/UdacityProject5/catkin_ws/devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" & 
sleep 5
xterm  -e  " source /home/workspace/UdacityProject5/catkin_ws/devel/setup.bash; roslaunch pick_objects pick_objects_demo.launch" & 
sleep 5