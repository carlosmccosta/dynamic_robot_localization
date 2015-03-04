/**\file pointcloud_conversions.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/common/common.h>
#include <dynamic_robot_localization/common/impl/pointcloud_conversions.hpp>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <template instantiations>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#ifndef DRL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>

#define PCL_INSTANTIATE_DRLPointcloudConversionsFromROSMsg(T) template bool dynamic_robot_localization::pointcloud_conversions::fromROSMsg<T>(const nav_msgs::OccupancyGrid&, pcl::PointCloud<T>&, dynamic_robot_localization::pointcloud_conversions::OccupancyGridValuesPtr, int);
PCL_INSTANTIATE(DRLPointcloudConversionsFromROSMsg, DRL_POINT_TYPES)

#define PCL_INSTANTIATE_DRLPointcloudConversionsFlipPointCloudNormalsUsingOccpancyGrid(T) template size_t dynamic_robot_localization::pointcloud_conversions::flipPointCloudNormalsUsingOccpancyGrid<T>(const nav_msgs::OccupancyGrid&, pcl::PointCloud<T>&, int, float, bool);
PCL_INSTANTIATE(DRLPointcloudConversionsFlipPointCloudNormalsUsingOccpancyGrid, DRL_POINT_TYPES)

#define PCL_INSTANTIATE_DRLPointcloudConversionsFromFile(T) template bool dynamic_robot_localization::pointcloud_conversions::fromFile< pcl::PointCloud<T> >(const std::string&, pcl::PointCloud<T>&);
PCL_INSTANTIATE(DRLPointcloudConversionsFromFile, DRL_POINT_TYPES)
PCL_INSTANTIATE(DRLPointcloudConversionsFromFile, DRL_DESCRIPTOR_TYPES)

#define PCL_INSTANTIATE_DRLPointcloudConversionsToFile(T) template bool dynamic_robot_localization::pointcloud_conversions::toFile< pcl::PointCloud<T> >(const std::string&, const pcl::PointCloud<T>&, bool);
PCL_INSTANTIATE(DRLPointcloudConversionsToFile, DRL_POINT_TYPES)
#endif


namespace dynamic_robot_localization {
namespace pointcloud_conversions {


template<>
bool fromFile(const std::string& filename, pcl::PCLPointCloud2& pointcloud) {
	std::string::size_type index = filename.rfind(".");
	if (index == std::string::npos) return false;

	std::string extension = filename.substr(index + 1);

	if (extension == "pcd") {
		if (pcl::io::loadPCDFile(filename, pointcloud) == 0 && !pointcloud.data.empty()) return true;
	} else if (extension == "ply") {
		if (pcl::io::loadPLYFile(filename, pointcloud) == 0 && !pointcloud.data.empty()) return true;
	} else {
		pcl::PolygonMesh mesh;
		if (pcl::io::loadPolygonFile(filename, mesh) != 0) { // obj | ply | stl | vtk | doesn't load normals curvature | doesn't load normals from .ply .stl
			pointcloud = mesh.cloud;
			return !pointcloud.data.empty();
		}
	}

	return false;
}


}
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </template instantiations>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
