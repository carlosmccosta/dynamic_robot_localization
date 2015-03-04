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
bool fromROSMsg(const nav_msgs::OccupancyGrid& occupancy_grid, pcl::PointCloud<PointT>& pointcloud, OccupancyGridValuesPtr occupancy_grid_values, int threshold_for_map_cell_as_obstacle) {
	if (occupancy_grid.data.size() > 0 && (occupancy_grid.data.size() == (occupancy_grid.info.width * occupancy_grid.info.height))) {
		float map_resolution = occupancy_grid.info.resolution;
		unsigned int map_width = occupancy_grid.info.width;
		unsigned int map_height = occupancy_grid.info.height;

		float map_origin_x = occupancy_grid.info.origin.position.x + map_resolution / 2.0;
		float map_origin_y = occupancy_grid.info.origin.position.y + map_resolution / 2.0;

		Eigen::Transform<float, 3, Eigen::Affine> transform =
				Eigen::Transform<float, 3, Eigen::Affine>(Eigen::Translation3f(map_origin_x, map_origin_y, 0)) *
				Eigen::Transform<float, 3, Eigen::Affine>(Eigen::Quaternionf(occupancy_grid.info.origin.orientation.w, occupancy_grid.info.origin.orientation.x, occupancy_grid.info.origin.orientation.y, occupancy_grid.info.origin.orientation.z));

		pointcloud.height = 1;
		pointcloud.is_dense = false;
		pointcloud.header.frame_id = occupancy_grid.header.frame_id;
		pointcloud.header.stamp = occupancy_grid.header.stamp.toNSec() / 1e3;
		pointcloud.clear();

		size_t data_position = 0;
		PointT new_point;
		for (unsigned int y = 0; y < map_height; ++y) {
			float x_map = 0.0;
			float y_map = (float)y * map_resolution;
			for (unsigned int x = 0; x < map_width; ++x) {
				if (occupancy_grid.data[data_position] > threshold_for_map_cell_as_obstacle) {
					x_map = (float)x * map_resolution;
					new_point.x = static_cast<float> (transform (0, 0) * x_map + transform (0, 1) * y_map + transform (0, 3));
					new_point.y = static_cast<float> (transform (1, 0) * x_map + transform (1, 1) * y_map + transform (1, 3));
					pointcloud.push_back(new_point);

					if (occupancy_grid_values) {
						occupancy_grid_values->push_back(occupancy_grid.data[data_position]);
					}
				}

				++data_position;
			}
		}

		pointcloud.width = pointcloud.size();
		return true;
	}

	return false;
}


template<typename PointT>
size_t flipPointCloudNormalsUsingOccpancyGrid(const nav_msgs::OccupancyGrid& occupancy_grid, pcl::PointCloud<PointT>& pointcloud, int search_k, float search_radius, bool show_occupancy_grid_pointcloud) {
	if (search_k <= 0 && search_radius <= 0) { return 0; }

	typename pcl::PointCloud<pcl::PointXY>::Ptr map_pointcloud(new pcl::PointCloud<pcl::PointXY>());
	OccupancyGridValuesPtr occupancy_grid_values(new OccupancyGridValues());
	if (fromROSMsg(occupancy_grid, *map_pointcloud, occupancy_grid_values, -2)) {
		if (show_occupancy_grid_pointcloud) {
			pcl::visualization::CloudViewer viewer("Cloud viewer");
			typename pcl::PointCloud<pcl::PointXYZI>::Ptr map_pointcloud_xyzi(new pcl::PointCloud<pcl::PointXYZI>());
			for (size_t i = 0; i < map_pointcloud->size(); ++i) {
				pcl::PointXYZI point;
				point.x = (*map_pointcloud)[i].x;
				point.y = (*map_pointcloud)[i].y;
				point.z = 0.0;
				point.intensity = ((*occupancy_grid_values)[i] < 0) ? 50 : (*occupancy_grid_values)[i];
				map_pointcloud_xyzi->push_back(point);
			}
			viewer.showCloud(map_pointcloud_xyzi);
			while (!viewer.wasStopped ()) {}
		}

		typename pcl::search::KdTree<pcl::PointXY>::Ptr map_pointcloud_search_method(new pcl::search::KdTree<pcl::PointXY>());
		map_pointcloud_search_method->setInputCloud(map_pointcloud);
		size_t number_of_flipped_normals = 0;

		for (size_t i = 0; i < pointcloud.size(); ++i) {
			PointT& current_point = pointcloud[i];
			pcl::PointXY current_point_xy;
			current_point_xy.x = current_point.x;
			current_point_xy.y = current_point.y;
			math_utils::PointPosition normal_side = math_utils::computePointPositionInRelationToLine(
					current_point.x, current_point.y,
					-current_point.normal_y, current_point.normal_x,
					current_point.x + current_point.normal_x, current_point.y + current_point.normal_y);

			math_utils::PointPosition points_distribution_side = math_utils::RIGHT_LINE_SIDE;
			size_t number_empty_cells_left = 0;
			size_t number_empty_cells_right = 0;

			pcl::IndicesPtr nn_indices(new std::vector<int>());
			std::vector<float> nn_distances;
			if (search_k > 0) {
				map_pointcloud_search_method->nearestKSearch(current_point_xy, search_k, *nn_indices, nn_distances);
			} else {
				map_pointcloud_search_method->radiusSearch(current_point_xy, search_radius, *nn_indices, nn_distances);
			}

			for (size_t nn_index = 0; nn_index < nn_indices->size(); ++nn_index) {
				pcl::PointXY& current_nn_point_xy = (*map_pointcloud)[(*nn_indices)[nn_index]];
				if ((*occupancy_grid_values)[(*nn_indices)[nn_index]] == 0) { // empty space in map
					math_utils::PointPosition point_side = math_utils::computePointPositionInRelationToLine(
							current_point.x, current_point.y,
							-current_point.normal_y, current_point.normal_x,
							current_nn_point_xy.x, current_nn_point_xy.y);
					if (point_side == math_utils::LEFT_LINE_SIDE) {
						++number_empty_cells_left;
					} else if (point_side == math_utils::RIGHT_LINE_SIDE) {
						++number_empty_cells_right;
					}
				}
			}

			if (number_empty_cells_left > number_empty_cells_right) {
				points_distribution_side = math_utils::LEFT_LINE_SIDE;
			}

			if (number_empty_cells_left != number_empty_cells_right && points_distribution_side != normal_side) {
				++number_of_flipped_normals;
				current_point.normal_x *= -1.0;
				current_point.normal_y *= -1.0;
				current_point.normal_z *= -1.0;
			}
		}

		return number_of_flipped_normals;
	}

	return 0;
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


template<typename PointCloudT>
bool toFile(const std::string& filename, const PointCloudT& pointcloud, bool save_in_binary_format) {
	std::string::size_type index = filename.rfind(".");
	if (index == std::string::npos) return false;

	std::string extension = filename.substr(index + 1);

	if (extension == "pcd") {
		if (pcl::io::savePCDFile(filename, pointcloud, save_in_binary_format) == 0) return true;
	} else if (extension == "ply") {
		if (pcl::io::savePLYFile(filename, pointcloud, save_in_binary_format) == 0) return true;
	}

	return false;
}



} /* namespace pointcloud_conversions */
} /* namespace dynamic_robot_localization */

