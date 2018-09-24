#!/usr/bin/env bash

catkin_ws_path=${1:-"$HOME/catkin_ws_drl"}
number_of_cpu_threads_for_compilation=${2:-2}
use_catkin_tools=${3:-true}
ros_version=${4:-"$(rosversion -d)"}
catkin_ws_path_to_extend=${5:-"/opt/ros/$ros_version"}


echo -e "\n\n"
echo "####################################################################################################"
echo "##### Installing the dynamic_robot_localization ROS package..."
echo "===== catkin_ws_path: ${catkin_ws_path}"
echo "===== number_of_cpu_threads_for_compilation: ${number_of_cpu_threads_for_compilation}"
echo "===== use_catkin_tools: ${use_catkin_tools}"
echo "===== ros_version: ${ros_version}"
echo "===== catkin_ws_path_to_extend: ${catkin_ws_path_to_extend}"
echo -e "####################################################################################################\n"

script_dir="$(dirname "$(readlink -e "${BASH_SOURCE[0]}")" && echo X)" && script_dir="${script_dir%$'\nX'}"

"${script_dir}/a_dependencies.bash" "${ros_version}"
"${script_dir}/b_workspace.bash" "${catkin_ws_path}" "${use_catkin_tools}" "${ros_version}" "${catkin_ws_path_to_extend}"
"${script_dir}/c_repositories.bash" "${catkin_ws_path}"
"${script_dir}/d_rosped.bash" "${catkin_ws_path}" "${ros_version}"
"${script_dir}/e_build.bash" "${catkin_ws_path}" "${number_of_cpu_threads_for_compilation}" "${use_catkin_tools}" "${ros_version}" "${catkin_ws_path_to_extend}"
