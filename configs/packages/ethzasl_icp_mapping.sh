#!/bin/sh

echo "####################################################################################################"
echo "##### Installing latest ethzasl_icp_mapping (http://wiki.ros.org/ethzasl_icp_mapping)"
echo "####################################################################################################"



echo "\n\n"
echo "----------------------------------------------------"
echo ">>>>> Cloning ethzasl_icp_mapping"
echo "----------------------------------------------------"
cd ~
mkdir Projects
cd Projects
git clone --recursive git://github.com/ethz-asl/ethzasl_icp_mapping.git


echo "\n\n"
echo "----------------------------------------------------"
echo ">>>>> Installing dependencies"
echo "----------------------------------------------------"
sudo apt-get install ros-groovy-map-msgs -y
sudo apt-get install libargtable2-dev -y
rosdep update
rosdep install ethzasl_icp_mapping



echo "\n\n"
echo "----------------------------------------------------"
echo ">>>>> Building ethzasl_icp_mapping"
echo "----------------------------------------------------"
rosmake ethzasl_icp_mapping