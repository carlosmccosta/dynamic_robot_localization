#!/bin/sh

# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Ubuntu 12.04 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


echo "####################################################################################################"
echo "##### Installing ROS Groovy (http://wiki.ros.org/groovy/Installation/Ubuntu)"
echo "####################################################################################################"

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-groovy-desktop-full -y

sudo rosdep init
rosdep update

echo "# <ROS groovy setup.bash>" >> ~/.bashrc
echo "export ROBOT=sim" >> ~/.bashrc
echo "export ROS_ENV_LOADER=/etc/ros/groovy/env.sh" >> ~/.bashrc
echo "source /opt/ros/groovy/setup.bash" >> ~/.bashrc
echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Projects" >> ~/.bashrc
echo "# </ROS groovy setup.bash>" >> ~/.bashrc
source /opt/ros/groovy/setup.bash

sudo apt-get install python-rosinstall -y
sudo apt-get install ros-groovy-openni-launch


echo "\n\n\n"
echo "####################################################################################################"
echo "##### Usage (http://gazebosim.org/wiki/Tutorials/1.9/Using_roslaunch_Files_to_Spawn_Models)"
echo "####################################################################################################"

echo "# roslaunch gazebo_worlds empty_world.launch"