#!/bin/sh

echo "####################################################################################################"
echo "##### Installing gazebo (http://gazebosim.org/wiki/1.9/install)"
echo "####################################################################################################"

# to remove Gazebo 1.5 that comes with ROS Groovy
# sudo apt-get remove ros-groovy-simulator-gazebo -y



echo "\n\n"
echo "------------------------------------------------"
echo ">>>>> Setting up sources.list"
echo "------------------------------------------------"

# >>> Ubuntu 12.04 (Precise)
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu precise main" > /etc/apt/sources.list.d/gazebo-latest.list'

# >>> Ubuntu 12.10 (Quantal)
# sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu quantal main" > /etc/apt/sources.list.d/gazebo-latest.list'

# >>> Ubuntu 13.04 (Raring)
# sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu raring main" > /etc/apt/sources.list.d/gazebo-latest.list'

# >>> Ubuntu 13.10 (Saucy)
# sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu saucy main" > /etc/apt/sources.list.d/gazebo-latest.list'



echo "\n\n"
echo "------------------------------------------------"
echo ">>>>> Setting keys"
echo "------------------------------------------------"
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -



echo "\n\n"
echo "------------------------------------------------"
echo ">>>>> Updating packages index"
echo "------------------------------------------------"
sudo apt-get update



echo "\n\n"
echo "------------------------------------------------"
echo ">>>>> Installing Gazebo"
echo "------------------------------------------------"
# gazebo 1.9
sudo apt-get install gazebo -y

# gazebo 2.2
# sudo apt-get install gazebo-current -y
