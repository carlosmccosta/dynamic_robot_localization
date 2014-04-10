#!/bin/sh

echo "####################################################################################################"
echo "##### Installing ROS Groovy (http://wiki.ros.org/groovy/Installation/Ubuntu)"
echo "####################################################################################################"


echo "\n\n"
echo "------------------------------------------------"
echo ">>>>> Setting up sources.list"
echo "------------------------------------------------"

# >>> Ubuntu 11.10 (Oneiric)
# sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu oneiric main" > /etc/apt/sources.list.d/ros-latest.list'

# >>> Ubuntu 12.04 (Precise)
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'

# >>> Ubuntu 12.10 (Quantal)
# sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu quantal main" > /etc/apt/sources.list.d/ros-latest.list'



echo "\n\n"
echo "------------------------------------------------"
echo ">>>>> Setting keys"
echo "------------------------------------------------"
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -



echo "\n\n"
echo "------------------------------------------------"
echo ">>>>> Updating packages index"
echo "------------------------------------------------"
sudo apt-get update



echo "\n\n"
echo "------------------------------------------------"
echo ">>>>> Installing ROS"
echo "------------------------------------------------"
# >>> Desktop-Full Install (Recommended)
sudo apt-get install ros-groovy-desktop-full -y

# >>> Desktop Install
# sudo apt-get install ros-groovy-desktop

# >>> ROS-Base (Bare Bones)
# sudo apt-get install ros-groovy-ros-base



echo "\n\n"
echo "------------------------------------------------"
echo ">>>>> Initializing rosdep"
echo "------------------------------------------------"
sudo rosdep init
rosdep update



echo "\n\n"
echo "------------------------------------------------"
echo ">>>>> Seting up environment"
echo "------------------------------------------------"
echo "# <ROS groovy setup.bash>" >> ~/.bashrc
echo "export ROBOT=sim" >> ~/.bashrc
echo "export ROS_ENV_LOADER=/etc/ros/groovy/env.sh" >> ~/.bashrc
echo "source /opt/ros/groovy/setup.bash" >> ~/.bashrc
# echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Projects" >> ~/.bashrc
echo "# </ROS groovy setup.bash>" >> ~/.bashrc
source ~/.bashrc



echo "\n\n"
echo "------------------------------------------------"
echo ">>>>> Installing remaining packages"
echo "------------------------------------------------"
sudo apt-get install python-rosinstall -y
sudo apt-get install ros-groovy-pcl-* -y
sudo apt-get install ros-groovy-openni-* -y
