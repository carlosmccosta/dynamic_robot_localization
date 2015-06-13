#!/usr/bin/env bash

catkin_ws=${1:-"$HOME/catkin_ws"}
ros_version=${2:-"$(rosversion -d)"}

script_dir="$(dirname "$(readlink -e "${BASH_SOURCE[0]}")" && echo X)" && script_dir="${script_dir%$'\nX'}"


source "/opt/ros/${ros_version}/setup.bash"
source "${catkin_ws}/devel/setup.bash"


"${script_dir}/a_dependencies.sh" ${ros_version}
"${script_dir}/b_repositories.sh" "${catkin_ws}"


echo -e "\n\n"
echo "--------------------------------------------------------------------"
echo "--- Installing remaining dependencies"

cd "${catkin_ws}"
rosdep update
rosdep check --from-paths src --ignore-src --rosdistro=${ros_version}
rosdep install --from-paths src --ignore-src --rosdistro=${ros_version}


echo -e "\n\n"
echo "--------------------------------------------------------------------"
echo "--- Dependencies that must be manually checked"

rosdep check --from-paths src --ignore-src --rosdistro=${ros_version}


echo -e "\n\n"
echo "####################################################################################################"
echo "##### Building catkin workspace"
echo "####################################################################################################"

cd "${catkin_ws}"
find ./src -name "*.sh" -exec chmod +x {} \;
find ./src -name "*.cfg" -exec chmod +x {} \;

catkin_make

echo -e "\n\n"
echo "----------------------------------------------------------------------------------------------------"
echo ">>>>> Finished building catkin workspace"
echo "----------------------------------------------------------------------------------------------------"
