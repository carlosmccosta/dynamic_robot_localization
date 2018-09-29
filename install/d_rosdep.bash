#!/usr/bin/env bash

catkin_ws_path=${1:-"$HOME/catkin_ws_drl"}
ros_version=${2:-"$(rosversion -d)"}

echo -e "\n\n"
echo "--------------------------------------------------------------------"
echo "--- Installing remaining dependencies"

cd "${catkin_ws_path}"
rosdep update
rosdep check --from-paths src --ignore-src --rosdistro="${ros_version}"
rosdep install --from-paths src --ignore-src --rosdistro="${ros_version}"


echo -e "\n\n"
echo "--------------------------------------------------------------------"
echo "--- Dependencies that must be manually checked"

rosdep check --from-paths src --ignore-src --rosdistro="${ros_version}"
