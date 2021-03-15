#pragma once

/**\file pointcloud_conversions.h
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// std includes
#include <memory>
#include <string>

// ROS includes
#include <ros/console.h>
#include <ros/publisher.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>

// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_representation.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>

// external libs includes
#include <Eigen/Core>
#include <Eigen/Geometry>

// project includes
#include <dynamic_robot_localization/common/math_utils.h>
#include <dynamic_robot_localization/common/pointcloud_utils.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


namespace dynamic_robot_localization {

// ##############################################################################   pointcloud_conversions   #############################################################################
namespace pointcloud_conversions {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <usings>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
using OccupancyGridValues = std::vector<signed char>;
using OccupancyGridValuesPtr = std::shared_ptr< OccupancyGridValues >;
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </usings>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

template <typename PointT>
bool fromROSMsg(const nav_msgs::OccupancyGrid& occupancy_grid, pcl::PointCloud<PointT>& pointcloud, OccupancyGridValuesPtr occupancy_grid_values = OccupancyGridValuesPtr(), int threshold_for_map_cell_as_obstacle = 95);

template <typename PointT>
bool publishPointCloud(pcl::PointCloud<PointT>& pointcloud, ros::Publisher& publisher, const std::string& frame_id, bool publish_pointcloud_only_if_there_is_subscribers, const std::string& point_cloud_name_for_logging);

template <typename PointT>
size_t flipPointCloudNormalsUsingOccpancyGrid(const nav_msgs::OccupancyGrid& occupancy_grid, pcl::PointCloud<PointT>& pointcloud, int search_k, float search_radius, bool show_occupancy_grid_pointcloud = false);

template <typename PointCloudT>
bool fromFile(PointCloudT& pointcloud, const std::string& filename, const std::string& folder = std::string(""));

template <typename PointCloudT>
bool toFile(const std::string& filename, const PointCloudT& pointcloud, bool save_in_binary_format, const std::string& folder = std::string(""));

} /* namespace pointcloud_conversions */
} /* namespace dynamic_robot_localization */


#ifdef DRL_NO_PRECOMPILE
#include <dynamic_robot_localization/common/impl/pointcloud_conversions.hpp>
#endif

