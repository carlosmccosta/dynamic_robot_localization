/**\file pointcloud_conversions.hpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/common/pointcloud_conversions.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {
namespace pointcloud_conversions {


template<typename PointT>
bool fromROSMsg(const nav_msgs::OccupancyGrid& occupancy_grid, pcl::PointCloud<PointT>& pointcloud, int threshold_for_map_cell_as_obstacle) {
	if (occupancy_grid.data.size() > 0 && (occupancy_grid.data.size() == (occupancy_grid.info.width * occupancy_grid.info.height))) {
		float map_resolution = occupancy_grid.info.resolution;
		unsigned int map_width = occupancy_grid.info.width;
		unsigned int map_height = occupancy_grid.info.height;

		float map_origin_x = occupancy_grid.info.origin.position.x + map_resolution / 2.0;
		float map_origin_y = occupancy_grid.info.origin.position.y + map_resolution / 2.0;

		pointcloud.height = 1;
		pointcloud.is_dense = false;
		pointcloud.header.frame_id = occupancy_grid.header.frame_id;
		pointcloud.header.stamp = occupancy_grid.header.stamp.toNSec() / 1e3;
		pointcloud.clear();

		size_t data_position = 0;
		PointT new_point;
		new_point.z = 0;
		for (unsigned int y = 0; y < map_height; ++y) {
			new_point.y = (float)y * map_resolution + map_origin_y;
			for (unsigned int x = 0; x < map_width; ++x) {
				if (occupancy_grid.data[data_position] > threshold_for_map_cell_as_obstacle) {
					new_point.x = (float)x * map_resolution + map_origin_x;
					pointcloud.push_back(new_point);
				}

				++data_position;
			}
		}

		pointcloud.width = pointcloud.size();
		return true;
	}

	return false;
}


template<typename PointCloudT>
bool fromFile(const std::string& filename, PointCloudT& pointcloud) {
	std::string::size_type index = filename.rfind(".");
	if (index == std::string::npos) return false;

	std::string extension = filename.substr(index + 1);

	if (extension == "pcd") {
		if (pcl::io::loadPCDFile(filename, pointcloud) == 0 && !pointcloud.empty()) return true;
	} else if (extension == "ply") {
		if (pcl::io::loadPLYFile(filename, pointcloud) == 0 && !pointcloud.empty()) {
			// fix PLYReader import
			pointcloud.sensor_origin_ = Eigen::Vector4f::Zero();
			pointcloud.sensor_orientation_ = Eigen::Quaternionf::Identity();
			return true;
		}
	} else {
		pcl::PolygonMesh mesh;
		if (pcl::io::loadPolygonFile(filename, mesh) != 0) { // obj | ply | stl | vtk | doesn't load normals curvature | doesn't load normals from .ply .stl
			pcl::fromPCLPointCloud2(mesh.cloud, pointcloud);
			return !pointcloud.empty();
		}
	}

	return false;
}



} /* namespace pointcloud_conversions */
} /* namespace dynamic_robot_localization */

