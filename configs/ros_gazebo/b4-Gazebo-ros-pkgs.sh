#!/bin/sh

echo "\n\n\n"
echo "####################################################################################################"
echo "##### Installing gazebo ros packages (http://www.gazebosim.org/wiki/Tutorials/1.9/Installing_gazebo_ros_Packages)"
echo "####################################################################################################"



echo "\n\n"
echo "----------------------------------------------------"
echo ">>>>> Installing gazebo ros packages for ROS Groovy"
echo "----------------------------------------------------"
cd ~/catkin_ws/src
git clone https://github.com/ros/cmake_modules.git
git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git
git clone https://github.com/ros-controls/ros_control.git
git clone https://github.com/ros-controls/ros_controllers.git -b groovy-backported-hydro
git clone https://github.com/ros-controls/control_toolbox.git
git clone https://github.com/ros-controls/realtime_tools.git


# echo "\n\n"
# echo "----------------------------------------------------"
# echo ">>>>> Installing gazebo ros packages for ROS Hydro"
# echo "----------------------------------------------------"
# sudo apt-get install ros-hydro-gazebo-ros-pkgs ros-hydro-gazebo-ros-control -y



echo "\n\n"
echo "----------------------------------------------------"
echo ">>>>> Installing dependencies"
echo "----------------------------------------------------"
rosdep update
rosdep check --from-paths . --ignore-src --rosdistro groovy
rosdep install --from-paths . --ignore-src --rosdistro groovy -y



echo "\n\n"
echo "----------------------------------------------------"
echo ">>>>> Building catkin workspace"
echo "----------------------------------------------------"
cd ~/catkin_ws/
catkin_make



echo "\n\n\n"
echo "####################################################################################################"
echo "##### Usage (http://gazebosim.org/wiki/Tutorials/1.9/Using_roslaunch_Files_to_Spawn_Models)"
echo "####################################################################################################"

echo "# Gazebo 1.5: roslaunch gazebo_worlds empty_world.launch"
echo "# Gazebo 1.9: roslaunch gazebo_ros empty_world.launch"
