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

echo "source /opt/ros/groovy/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt-get install python-rosinstall -y


echo "\n\n\n"
echo "####################################################################################################"
echo "##### Creating ROS catkin workspace (http://wiki.ros.org/catkin/Tutorials/create_a_workspace)"
echo "####################################################################################################"

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace

cd ~/catkin_ws/
catkin_make

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc



echo "\n\n\n"
echo "####################################################################################################"
echo "##### Installing gazebo (http://gazebosim.org/wiki/1.9/install)"
echo "####################################################################################################"

sudo apt-get remove ros-groovy-simulator-gazebo -y


sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu precise main" > /etc/apt/sources.list.d/gazebo-latest.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update

# gazebo 2.2
#sudo apt-get install gazebo-current -y

# gazebo 1.9
sudo apt-get install gazebo -y



echo "\n\n\n"
echo "####################################################################################################"
echo "##### Installing gazebo ros packages (http://www.gazebosim.org/wiki/Tutorials/1.9/Installing_gazebo_ros_Packages)"
echo "####################################################################################################"

# hydro
# sudo apt-get install ros-hydro-gazebo-ros-pkgs ros-hydro-gazebo-ros-control -y


# groovy
cd ~/catkin_ws/src
git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git
git clone https://github.com/ros-controls/ros_control.git
git clone https://github.com/ros-controls/ros_controllers.git -b groovy-backported-hydro
git clone https://github.com/ros-controls/control_toolbox.git
git clone https://github.com/ros-controls/realtime_tools.git

rosdep update
rosdep check --from-paths . --ignore-src --rosdistro groovy
rosdep install --from-paths . --ignore-src --rosdistro groovy -y

cd ~/catkin_ws/
catkin_make



echo "\n\n\n"
echo "####################################################################################################"
echo "##### Usage (http://gazebosim.org/wiki/Tutorials/1.9/Using_roslaunch_Files_to_Spawn_Models)"
echo "####################################################################################################"

echo "# roslaunch gazebo_ros empty_world.launch"
echo "# roscore & rosrun gazebo_ros gazebo"