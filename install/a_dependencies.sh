#!/usr/bin/env sh

ros_version=${1:-"$(rosversion -d)"}


echo "####################################################################################################"
echo "##### Checking and installing dependencies for dynamic_robot_localization ros package"
echo "####################################################################################################"

sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl -y
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get dist-upgrade -y

# required system dependencies
sudo apt-get install coreutils -y
sudo apt-get install git -y
sudo apt-get install libpcl-1.7-all-dev -y
sudo apt-get install libboost-all-dev -y
sudo apt-get install python-wstool -y


# required ros packages
sudo apt-get install ros-${ros_version}-angles -y
sudo apt-get install ros-${ros_version}-cmake-modules -y
sudo apt-get install ros-${ros_version}-cpp-common -y
sudo apt-get install ros-${ros_version}-dynamic-reconfigure -y
sudo apt-get install ros-${ros_version}-geometry-msgs -y
sudo apt-get install ros-${ros_version}-map-server -y
sudo apt-get install ros-${ros_version}-message-generation -y
sudo apt-get install ros-${ros_version}-message-runtime -y
sudo apt-get install ros-${ros_version}-pcl-conversions -y
sudo apt-get install ros-${ros_version}-pcl-ros -y
sudo apt-get install ros-${ros_version}-rospp -y
sudo apt-get install ros-${ros_version}-rosconsole -y
sudo apt-get install ros-${ros_version}-rostime -y
sudo apt-get install ros-${ros_version}-sensor-msgs -y
sudo apt-get install ros-${ros_version}-std-msgs -y
sudo apt-get install ros-${ros_version}-tf2 -y
sudo apt-get install ros-${ros_version}-tf2-ros -y
sudo apt-get install ros-${ros_version}-xmlrpcpp -y


# optional ros packages for ukf filtering
sudo apt-get install ros-${ros_version}-robot-localization -y


# optional ros packages for dynamic map update
sudo apt-get install ros-${ros_version}-octomap -y
sudo apt-get install ros-${ros_version}-octomap-ros -y
sudo apt-get install ros-${ros_version}-octomap-msgs -y
sudo apt-get install ros-${ros_version}-octomap-rviz-plugins -y
sudo apt-get install ros-${ros_version}-dynamic-edt-3d -y
sudo apt-get install ros-${ros_version}-octovis -y


# optional system dependencies for mesh_to_pointcloud package
sudo apt-get install libopenthreads-dev -y
sudo apt-get install libopenscenegraph-dev -y


echo "\n\n"
echo "----------------------------------------------------------------------------------------------------"
echo ">>>>> Installation of drl dependencies finished"
echo "----------------------------------------------------------------------------------------------------"
