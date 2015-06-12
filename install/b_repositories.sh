#!/usr/bin/env sh

echo "####################################################################################################"
echo "##### Cloning git repositories"
echo "####################################################################################################"

catkin_ws=${1:-'$HOME/catkin_ws/src/'}
repositories=${2:-"dynamic_robot_localization pose_to_tf_publisher laserscan_to_pointcloud octomap_mapping mesh_to_pointcloud"}

cd "${catkin_ws}"
if [ $? -ne 0 ]; then
	exit -1
fi

for git_repository in ${repositories}
do
	ls "${git_repository}" &> /dev/null
	if [ $? -ne 0 ]; then
		echo -e "\n\n"
		echo "-------------------------------------------"
		echo "==> Cloning ${git_repository}"
		git clone "https://github.com/carlosmccosta/${git_repository}.git"
	fi
done


echo -e "\n\n"
echo "----------------------------------------------------------------------------------------------------"
echo ">>>>> Cloning git repositories finished"
echo ">>>>> For updating each git repository use: git pull"
echo ">>>>> Or run ${catkin_ws}/dynamic_robot_localization/install/c_repositories_update.sh to update all repositories"
echo "----------------------------------------------------------------------------------------------------"
