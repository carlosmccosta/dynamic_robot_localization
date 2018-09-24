#!/usr/bin/env bash

reset_ws=${1:-true}
catkin_ws_path=${2:-"$HOME/catkin_ws_drl"}
repositories_dirs=${3:-"dynamic_robot_localization pose_to_tf_publisher laserscan_to_pointcloud octomap_mapping mesh_to_pointcloud pcl pcl_msgs perception_pcl"}


echo -e "\n\n"
echo "####################################################################################################"
echo "##### Updating git repositories..."
if [ "${reset_ws}" = true ] ; then
	echo "##### git reset --hard HEAD will be performed before pulling"
fi
echo -e "####################################################################################################\n"


cd "${catkin_ws_path}/src/" &> /dev/null
if [ $? -ne 0 ]; then
	echo "==============================================================="
	echo -e "==> Missing workspace directory (${catkin_ws_path}/src/)!\n\n"
	exit -1
fi


for git_repository in ${repositories_dirs}
do
	cd "${git_repository}" &> /dev/null
	if [ $? -eq 0 ]; then
		echo -e "\n\n-------------------------------------------"
		echo "==> Updating ${git_repository}"
		if [ "${reset_ws}" = true ] ; then
			git reset --hard HEAD
		fi
		git pull
		cd ..
	fi
done

cd "${catkin_ws_path}"
find ./src -name "*.bash" -exec chmod +x {} \;
find ./src -name "*.cfg" -exec chmod +x {} \;
find ./src -name "*.sh" -exec chmod +x {} \;


echo -e "\n\n"
echo "----------------------------------------------------------------------------------------------------"
echo ">>>>> Update finished"
echo "----------------------------------------------------------------------------------------------------"

