#!/usr/bin/env bash

echo "####################################################################################################"
echo "##### Initializing catkin workspace..."
echo "####################################################################################################"

catkin_ws_path=${1:-"$HOME/catkin_ws_drl"}
use_catkin_tools=${2:-true}
ros_version=${3:-"$(rosversion -d)"}
catkin_ws_path_to_extend=${4:-"/opt/ros/$ros_version"}


function AddProfile {
	catkin config --profile $1 -x _$1 --cmake-args -DCMAKE_BUILD_TYPE=$2
	catkin profile set $1
	catkin config --extend $3
	catkin config --append-args --cmake-args -DCMAKE_CXX_FLAGS="-Wall -Wextra"
}

source "${catkin_ws_path_to_extend}/setup.bash"

ls "${catkin_ws_path}/src" &> /dev/null
if [ $? -ne 0 ]; then
	mkdir -p "${catkin_ws_path}/src"
fi

ws_initialized=false
if [ $use_catkin_tools = true ]; then
	ls "${catkin_ws_path}/.catkin_tools" &> /dev/null
	if [ $? -ne 0 ]; then
		echo "==========================================="
		echo "==> Initializing catkin tools workspace..."
		cd "${catkin_ws_path}"
		catkin config --init --no-install --extend ${catkin_ws_path_to_extend}

		echo "==========================================="
		echo "==> Creating Debug profile..."
		AddProfile debug Debug ${catkin_ws_path_to_extend}

		echo "==========================================="
		echo "==> Creating RelWithDebInfo profile..."
		AddProfile release_with_debug_info RelWithDebInfo ${catkin_ws_path_to_extend}

		echo "==========================================="
		echo "==> Creating Release profile..."
		AddProfile release Release ${catkin_ws_path_to_extend}
		ws_initialized=true
	fi
else
	ls "${catkin_ws_path}/src/CMakeLists.txt" &> /dev/null
	if [ $? -ne 0 ]; then
		echo "==> Initializing catkin workspace..."
		cd "${catkin_ws_path}/src"
		catkin_init_workspace
		ws_initialized=true
	fi
fi

if [ $ws_initialized = true ]; then
	echo "--------------------------------------------------------"
	echo "==> Finished workspace initialization"
	echo "--------------------------------------------------------"
else
	echo "--------------------------------------------------------"
	echo "==> Workspace already exists. Skipping initialization..."
	echo "--------------------------------------------------------"
fi
