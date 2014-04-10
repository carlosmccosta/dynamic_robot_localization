#!/bin/sh

echo "####################################################################################################"
echo "##### Installing RViz"
echo "####################################################################################################"



echo "\n\n"
echo "----------------------------------------------------"
echo ">>>>> Cloning RViz groovy-devel"
echo "----------------------------------------------------"
cd ~
mkdir Projects
cd Projects
git clone https://github.com/ros-visualization/rviz.git -b groovy-devel


echo "\n\n"
echo "----------------------------------------------------"
echo ">>>>> Building catkin workspace"
echo "----------------------------------------------------"
cd rviz
mkdir build
cd build
cmake ..
make

echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Projects" >> ~/.bashrc
echo "source ~/Projects/rviz/build/devel/setup.bash" >> ~/.bashrc
