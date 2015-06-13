#!/usr/bin/env bash

echo "####################################################################################################"
echo "##### Creating eclipse project for package dynamic_robot_localization"
echo "####################################################################################################"


mkdir -p ~/catkin_ws/build/dynamic_robot_localization
cd ~/catkin_ws/build/dynamic_robot_localization
cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE -DCMAKE_BUILD_TYPE=Debug -DCMAKE_ECLIPSE_MAKE_ARGUMENTS=-j8 ~/catkin_ws/src/dynamic_robot_localization


echo -e "\n\n"
echo "----------------------------------------------------------------------------------------------------"
echo ">>>>> Eclipse build  project in ~/catkin_ws/build/dynamic_robot_localization"
echo ">>>>> Eclipse source project in ~/catkin_ws/src/dynamic_robot_localization"
echo "----------------------------------------------------------------------------------------------------"



echo -e "\n\n\n\n"
echo "####################################################################################################"
echo "##### Creating eclipse projects for catkin workspace"
echo "####################################################################################################"

cd ~/catkin_ws
catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_ECLIPSE_MAKE_ARGUMENTS=-j8


echo -e "\n\n"
echo "----------------------------------------------------------------------------------------------------"
echo ">>>>> Eclipse catkin_ws project in ~/catkin_ws/build"
echo "----------------------------------------------------------------------------------------------------"
