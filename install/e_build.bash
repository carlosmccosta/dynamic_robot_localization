#!/usr/bin/env bash

catkin_ws_path=${1:-"$HOME/catkin_ws_drl"}
number_of_cpu_threads_for_compilation=${2:-2}
use_catkin_tools=${3:-true}
ros_version=${4:-"$(rosversion -d)"}
catkin_ws_path_to_extend=${5:-"/opt/ros/$ros_version"}


echo -e "\n\n"
echo "####################################################################################################"
echo "##### Building catkin workspace"
echo "####################################################################################################"

cd "${catkin_ws_path}"
source "${catkin_ws_path_to_extend}/setup.bash"

if [ $use_catkin_tools = true ]; then
	devel_file="${catkin_ws_path}/devel_release/setup.bash"
	ws_overlay_url="https://catkin-tools.readthedocs.io/en/latest/mechanics.html#workspace-chaining-extending"
	catkin build PCL -s -j${number_of_cpu_threads_for_compilation}
	source ${devel_file}
	catkin build -s -j${number_of_cpu_threads_for_compilation}
else
	ws_overlay_url="http://wiki.ros.org/catkin/Tutorials/workspace_overlaying"
	devel_file="${catkin_ws_path}/devel_isolated/setup.bash"
	catkin_make_isolated --pkg PCL --force-cmake -DCMAKE_BUILD_TYPE=Release -j${number_of_cpu_threads_for_compilation}
	source ${devel_file}
	catkin_make_isolated -DCMAKE_BUILD_TYPE=Release -j${number_of_cpu_threads_for_compilation}
fi

source ${devel_file}
rospack profile
echo "source ${devel_file}" >> ~/.bashrc

echo -e "\n\n"
echo "----------------------------------------------------------------------------------------------------"
echo "===== Finished building the catkin workspace"
echo "----- For using this workspace source the file ${devel_file}"
echo "----- For overlaying / extending this workspace please read the following documentation:"
echo "----- ${ws_overlay_url}"
echo "----- Thank you for using the dynamic_robot_localization package!"
echo "----- Suggestions for improvements and pull requests are always welcomed!"
echo "----------------------------------------------------------------------------------------------------"
