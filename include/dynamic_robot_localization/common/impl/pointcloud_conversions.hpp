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

		float map_origin_x = occupancy_grid.info.origin.position.x;
		float map_origin_y = occupancy_grid.info.origin.position.y;

		pointcloud.height = 1;
		pointcloud.is_dense = false;
		pointcloud.header.frame_id = occupancy_grid.header.frame_id;
		pointcloud.header.stamp = occupancy_grid.header.stamp.toNSec() / 1e3;
		pointcloud.points.clear();

		size_t data_position = 0;
		PointT new_point;
		new_point.z = 0;
		for (unsigned int y = 0; y < map_height; ++y) {
			new_point.y = (float)y * map_resolution + map_origin_y;
			for (unsigned int x = 0; x < map_width; ++x) {
				if (occupancy_grid.data[data_position] > threshold_for_map_cell_as_obstacle) {
					new_point.x = (float)x * map_resolution + map_origin_x;
					pointcloud.points.push_back(new_point);
				}

				++data_position;
			}
		}

		pointcloud.width = pointcloud.points.size();
		return true;
	}

	return false;
}


template<typename PointT>
bool fromFile(const std::string& filename, pcl::PointCloud<PointT>& pointcloud) {
	std::string::size_type index = filename.rfind(".");
	if (index == std::string::npos) return false;

	std::string extension = filename.substr(index + 1);

	if (extension == "pcd") {
		if (pcl::io::loadPCDFile<PointT>(filename, pointcloud) == 0 && !pointcloud.points.empty()) return true;
	} else {
		pcl::PolygonMesh mesh;
		if (pcl::io::loadPolygonFile(filename, mesh) != 0) { // obj | ply | stl | vtk | doesn't load normals curvature
			pcl::fromPCLPointCloud2(mesh.cloud, pointcloud);
			return !pointcloud.points.empty();
		}
	}

	return false;
}



} /* namespace pointcloud_conversions */
} /* namespace dynamic_robot_localization */

