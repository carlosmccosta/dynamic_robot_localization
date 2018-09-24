#!/usr/bin/env bash

echo "####################################################################################################"
echo "##### Cloning git repositories..."
echo "####################################################################################################"

catkin_ws_path=${1:-"$HOME/catkin_ws_drl"}
repositories=${2:-$(cat <<-END
	https://github.com/carlosmccosta|dynamic_robot_localization
	https://github.com/carlosmccosta|pose_to_tf_publisher
	https://github.com/carlosmccosta|laserscan_to_pointcloud
	https://github.com/carlosmccosta|octomap_mapping
	https://github.com/carlosmccosta|mesh_to_pointcloud
	https://github.com/carlosmccosta|pcl
	https://github.com/ros-perception|pcl_msgs
	https://github.com/ros-perception|perception_pcl
END
)
}


cd "${catkin_ws_path}/src/" &> /dev/null
if [ $? -ne 0 ]; then
	mkdir -p "${catkin_ws_path}/src"
	cd "${catkin_ws_path}/src/"
fi


ls "${catkin_ws_path}/src/.rosinstall" &> /dev/null
if [ $? -ne 0 ]; then
	wstool init
fi

for git_repository in ${repositories}
do
	IFS="|"; set -- ${git_repository};
	ls "${2}" &> /dev/null
	if [ $? -ne 0 ]; then
		git_repository_url="${1}/${2}.git"
		echo -e "\n\n"
		echo "-------------------------------------------"
		echo "==> Cloning ${2} from ${git_repository_url}"
		git clone ${git_repository_url}
		wstool set "${2}" "${git_repository_url}" --git -y
	else
		echo -e "\n\n"
		echo "-------------------------------------------"
		echo "==> Updating ${2}"
		cd ${2}
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
echo ">>>>> Cloning git repositories finished"
echo ">>>>> For updating each git repository use: git pull"
echo ">>>>> For updating all repositories use:"
echo ">>>>> ${catkin_ws_path}/src/dynamic_robot_localization/install/repositories_update.sh"
echo ">>>>> or"
echo ">>>>> cd ${catkin_ws_path}/src"
echo ">>>>> wstool status"
echo ">>>>> Commit or stash modified files"
echo ">>>>> wstool update"
echo "----------------------------------------------------------------------------------------------------"
