/**\file pointcloud_utils.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/common/common.h>
#include <dynamic_robot_localization/common/impl/pointcloud_utils.hpp>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <template instantiations>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#ifndef DRL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>

#define PCL_INSTANTIATE_DRLPointCloudUtilsConcatenatePointClouds(T) template void dynamic_robot_localization::pointcloud_utils::concatenatePointClouds<T>(const std::vector< typename pcl::PointCloud<T>::Ptr >&, typename pcl::PointCloud<T>::Ptr&);
PCL_INSTANTIATE(DRLPointCloudUtilsConcatenatePointClouds, DRL_POINT_TYPES)

#define PCL_INSTANTIATE_DRLPointCloudUtilsColorizePointCloudWithCurvature(T) template void dynamic_robot_localization::pointcloud_utils::colorizePointCloudWithCurvature<T>(pcl::PointCloud<T>&);
PCL_INSTANTIATE(DRLPointCloudUtilsColorizePointCloudWithCurvature, DRL_POINT_TYPES)

#define PCL_INSTANTIATE_DRLPointCloudUtilsColorizePointCloudClusters(T) template void dynamic_robot_localization::pointcloud_utils::colorizePointCloudClusters<T>(const pcl::PointCloud<T>&, const std::vector<pcl::PointIndices>&, pcl::PointCloud<T>&);
PCL_INSTANTIATE(DRLPointCloudUtilsColorizePointCloudClusters, DRL_POINT_TYPES)

#define PCL_INSTANTIATE_DRLPointCloudUtilsExtractPointCloudClusters(T) template void dynamic_robot_localization::pointcloud_utils::extractPointCloudClusters<T>(const pcl::PointCloud<T>&, const std::vector<pcl::PointIndices>&, const std::vector<size_t>&, pcl::PointCloud<T>&);
PCL_INSTANTIATE(DRLPointCloudUtilsExtractPointCloudClusters, DRL_POINT_TYPES)

#define PCL_INSTANTIATE_DRLPointCloudUtilsDistanceSquaredToOrigin(T) template float dynamic_robot_localization::pointcloud_utils::distanceSquaredToOrigin<T>(const T&);
PCL_INSTANTIATE(DRLPointCloudUtilsDistanceSquaredToOrigin, DRL_POINT_TYPES)

#endif
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </template instantiations>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
