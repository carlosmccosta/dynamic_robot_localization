#!/bin/sh

echo "####################################################################################################"
echo "##### Installing RViz"
echo "####################################################################################################"



echo "\n\n"
echo "----------------------------------------------------"
echo ">>>>> Cloning RViz groovy-devel"
echo "----------------------------------------------------"
cd ~/catkin_ws/src
git clone https://github.com/ros-visualization/rviz.git -b groovy-devel



echo "\n\n"
echo "----------------------------------------------------"
echo ">>>>> Building catkin workspace"
echo "----------------------------------------------------"
cd ~/catkin_ws/
catkin_make --pkg rviz
