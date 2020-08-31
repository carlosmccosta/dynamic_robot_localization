#pragma once

/**\file pointcloud_utils.h
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// std includes
#include <limits>
#include <vector>

// PCL includes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types_conversion.h>
#include <pcl/PointIndices.h>
#include <pcl/common/colors.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {
// ############################################################################   pointcloud_utils   ###########################################################################

namespace pointcloud_utils {

template <typename PointT>
void concatenatePointClouds(const std::vector< typename pcl::PointCloud<PointT>::Ptr >& pointclouds, typename pcl::PointCloud<PointT>::Ptr& pointcloud_out);

template <typename PointT>
void colorizePointCloudWithCurvature(pcl::PointCloud<PointT>& pointcloud);

template <typename PointT>
void colorizePointCloudClusters(const pcl::PointCloud<PointT>& pointcloud, const std::vector<pcl::PointIndices>& cluster_indices, pcl::PointCloud<PointT>& pointcloud_colored_out);

template <typename PointT>
void extractPointCloudClusters(const pcl::PointCloud<PointT>& pointcloud, const std::vector<pcl::PointIndices>& cluster_indices, const std::vector<size_t>& selected_clusters, pcl::PointCloud<PointT>& pointcloud_out);

template <typename PointT>
float distanceSquaredToOrigin(const PointT& point);

std::string getFileExtension(const std::string& filename);

std::string parseFilePath(const std::string& filename, const std::string& folder);

} /* namespace pointcloud_utils */
} /* namespace dynamic_robot_localization */


#ifdef DRL_NO_PRECOMPILE
#include <dynamic_robot_localization/common/impl/pointcloud_utils.hpp>
#endif

