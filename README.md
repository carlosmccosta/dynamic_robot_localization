[![DOI](https://zenodo.org/badge/7410/carlosmccosta/dynamic_robot_localization.png)](http://dx.doi.org/10.5281/zenodo.12907)


Dynamic Robot Localization
==================


## Overview

The dynamic_robot_localization is a ROS package that offers 3 DOF and 6 DOF localization using PCL and allows dynamic map update using OctoMap.
It's a modular localization pipeline, that can be configured using yaml files (detailed configuration layout available in [drl_configs.yaml](https://github.com/carlosmccosta/dynamic_robot_localization/blob/hydro-devel/yaml/schema/drl_configs.yaml))

[![Localization system tests with Guardian platform in CROB lab](http://img.youtube.com/vi/Rl6utteku8k/maxresdefault.jpg)](http://www.youtube.com/watch?v=Rl6utteku8k)


## Data sources

The localization system can operate with any type of sensors that provide point clouds.
As such, it can use any number of laser sensors (by using the [laserscan_to_pointcloud](https://github.com/carlosmccosta/laserscan_to_pointcloud) package) or RGB-D / ToF cameras.



## Reference map

The reference map can be provided to the localization system from CAD ( .ply | .stl | .obj | .vtk ), pointcloud file (.pcd) / topic (sensor_msgs::PointCloud2) or using a 2D costmap (nav_msgs::OccupancyGrid).
To avoid unnecessary overhead it should be provided has pointcloud (.pcd).

To convert CAD files to .pcd there are two options:

1. Using [conversion scripts](https://github.com/carlosmccosta/dynamic_robot_localization/tree/hydro-devel/tools) that filter the CAD using meshlabserver, add point curvature information and convert the CAD to .pcd (recommended)
 - Supported file types: .3ds .aln .apts .asc .dae .gts .obj .off .ply .pts .ptx .stl .tri .v3d .vrml .x3d .x3dv .xyz
2. Using the [mesh_to_pointcloud](https://github.com/carlosmccosta/mesh_to_pointcloud) package that converts CAD to .pcd directly (without curvature and filtering)
 - Supported file types: .3dc .3ds .asc .ac .bsp .dae .dw .dxf .fbx .flt .gem .geo .iv .ive .logo .lwo .lw .lws .md2 .obj .ogr .osg .pfb .ply .shp .stl .x .wrl



## Localization methods

The localization system has two different localization methods to tackle different scenarios.
For initial pose estimation and pose recovery it can use feature matching with several keypoint detectors (such as Scale Invariant Feature Transform (SIFT) and Intrinsic Shape Signatures (ISS3D)) and several keypoint descriptors (such as Point Feature Histogram (PFH), Fast Point Feature Histogram (FPFH), Signature of Histograms of Orientations (SHOT), Shape Context 3D (SC3D), Unique Shape Context (USC) and Ensemble of Shape Functions (ESF)).
For pose tracking it can use several variants of the Iterative Closest Point algorithm and also the Normal Distributions Transform (NDT).
Moreover, pose tracking can have two configurations. One tuned for the expected operation conditions of the robot and another able to recover from odometry estimation problems (the system can operate without odometry).
This gives the possibility to have a configuration that can perform pose tracking with the minimum amount of computational resources required (such as point-to-point ICP) and a more robust (and computational expensive) configuration to handle temporary tracking problems (such as point-to-point non linear ICP, point-to-plane ICP, generalized ICP).
Examples of the localization pipeline are available at [dynamic_robot_localization/yaml/configs](https://github.com/carlosmccosta/dynamic_robot_localization/tree/hydro-devel/yaml/configs)



## Localization reliability

The localization pipeline allows the attachment of transformation validators before publishing pose estimations.
They can be used to suppress estimations that are not plausible for a given robot operation conditions.
These validators can suppress localization estimations in which the pose correction has a [ outlier percentage | inliers root mean square error | inliers / outliers angular distribution | translation / rotation corrections ] outside acceptable thresholds.
If the tracking methods can't recover after a given amount of sensor updates / time, the initial pose estimation (using features) can be activated.



## Dynamic map update

Dynamic map update can be performed with the [OctoMap package](http://octomap.github.io/)
This can be [achieved](https://github.com/carlosmccosta/dynamic_robot_localization/blob/hydro-devel/launch/octo_map.launch) by configuring either the registered cloud or the outlier cloud as input for OctoMap and the output of OctoMap as the reference cloud for the localization system.



## Usage

The localization system can be started with [dynamic_robot_localization_system.launch](https://github.com/carlosmccosta/dynamic_robot_localization/blob/hydro-devel/launch/dynamic_robot_localization_system.launch)

This launch file starts the nodes:

* dynamic_robot_localization
* pose_to_tf_publisher (can be disabled)
* laserscan_to_pointcloud_assembler (can be disabled)
* octomap (can be disabled)

The localization is integrated with other ROS nodes by publishing TF transforms between map and odom frames (configurable frame names) using the [pose_to_tf_publisher node](https://github.com/carlosmccosta/pose_to_tf_publisher).
The dynamic_robot_localization node publishes geometry_msgs::PoseStamped / geometry_msgs::PoseWithCovarianceStamped that are received by pose_to_tf_publisher and converted to TF messages.
The TF is published in a separate ROS node to ensure that the system remains operational and with constant TF publish rate even if the localization node has variable computational time.
Moreover, in the remote case that the localization system crashes (by lack of system resources, programming error in the localization node or in one of its external libraries), the system can remain operational using only odometry for a given amount of time (i.e. TF messages can be published continuously, with a timeout counted after the last valid pose estimation or one message for each pose estimation) and the localization node can be restarted without any impact in the rest of the system.

The dynamic_robot_localization node publishes in latched topics (if required, as the publish of these messages can be disabled to reduce consumption of system resources):

1. dynamic_robot_localization::LocalizationDetailed
 - Message that has detailed information about the localization estimation (number of registered points | number of inliers | outlier percentage | inliers root mean square error | inliers / outliers angular distribution | translation / rotation corrections | global estimation accepted poses)
2. dynamic_robot_localization::LocalizationTimes
 - Message with the computation time of each localization processing stage (and also the global time)
3. dynamic_robot_localization::LocalizationDiagnostics
 - Message with diagnostics (currently only has the point clouds size after each major localization pipeline stages)
4. Registered clouds (sensor_msgs::PointCloud2)
 - The full registered cloud or its points categorized as inliers / outliers (computed in each pose estimation) is also published as 3 separated point clouds.
5. Reference point cloud (sensor_msgs::PointCloud2)
 - The current reference point cloud is also published in order to be available to localization supervisors



## Dynamic robot localization tests

The localization system is being tested using two different robot platforms (Guardian and Jarvis), both in simulation and with the real robots in the INESC CROB lab.

The guardian platform uses the Gazebo simulator with the [guardian-ros-pkg](https://github.com/inesc/guardian-ros-pkg)
The jarvis platform uses the Stage simulator.

Besides the CROB lab environment, the guardian platform was also tested in a ship interior with different configurations (with increasing level of difficulty for the localization system):

* static
* static with unstable ground
* static with cluttered environment
* dynamic with cluttered environment

The testing configurations are managed in [localization_tests.launch](https://github.com/carlosmccosta/dynamic_robot_localization_tests/blob/hydro-devel/launch/localization_tests.launch)

They can use live data from the gazebo simulator or using [rosbags](https://github.com/carlosmccosta/dynamic_robot_localization_tests/tree/hydro-devel/datasets)

The test results along with environment screenshots / videos are available in [this shared folder](https://www.dropbox.com/sh/nwb6gezj2dan187/AABM2u4BGd12lN__nYFwSktLa?dl=0)



# Installation and package dependencies

Installation scripts can be found in this [shared folder.](https://www.dropbox.com/sh/fkio31gyt55oolj/AACGFB2gB5eH-o8B0Cc4qeHma?dl=0)



## List of related git repositories:

* [dynamic_robot_localization_tests](https://github.com/carlosmccosta/dynamic_robot_localization_tests)
* [pose_to_tf_publisher](https://github.com/carlosmccosta/pose_to_tf_publisher)
* [laserscan_to_pointcloud](https://github.com/carlosmccosta/laserscan_to_pointcloud)
* [mesh_to_pointcloud](https://github.com/carlosmccosta/mesh_to_pointcloud)
* [robot_localization_tools](https://github.com/carlosmccosta/robot_localization_tools)
* [crob_gazebo_models](https://github.com/carlosmccosta/crob_gazebo_models)
* [octomap_mapping](https://github.com/carlosmccosta/octomap_mapping)



## More info

* [ICIT 2015 paper](https://www.dropbox.com/sh/yizj93xtvsapl9e/AABdCPKrMX2V58vzpzECKiExa?dl=0)
* [Results folder](https://www.dropbox.com/sh/nwb6gezj2dan187/AABM2u4BGd12lN__nYFwSktLa?dl=0)
* [Dissertation webpage](http://carlosmccosta.wix.com/personal-webpage#!dissertation/c12dl)
* [Dissertation abstract](http://1drv.ms/1odZRYO)
* [Research](http://1drv.ms/1l8yGei)
