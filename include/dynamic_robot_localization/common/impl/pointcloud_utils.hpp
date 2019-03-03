/**\file pointcloud_utils.hpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/common/pointcloud_utils.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {
namespace pointcloud_utils {


template<typename PointT>
void concatenatePointClouds(const std::vector< typename pcl::PointCloud<PointT>::Ptr >& pointclouds, typename pcl::PointCloud<PointT>::Ptr& pointcloud_out) {
	if (pointcloud_out) {
		for (size_t i = 0; i < pointclouds.size(); ++i) {
			if (pointclouds[i] && !pointclouds[i]->empty()) {
				*pointcloud_out += *(pointclouds[i]);
			}
		}
	}
}


template <typename PointT>
void colorizePointCloudWithCurvature(pcl::PointCloud<PointT>& pointcloud) {
	float min_curvature = std::numeric_limits<float>::max();
	float max_curvature = std::numeric_limits<float>::min();

	for (size_t i = 0; i < pointcloud.size(); ++i) {
		if (pointcloud[i].curvature < min_curvature)
			min_curvature = pointcloud[i].curvature;

		if (pointcloud[i].curvature > max_curvature)
			max_curvature = pointcloud[i].curvature;
	}

	float curvature_scale = 360.0f / (max_curvature - min_curvature);

	for (size_t i = 0; i < pointcloud.size(); ++i) {
		pcl::PointXYZHSV hsv;
		hsv.h = (pointcloud[i].curvature - min_curvature) * curvature_scale;
		hsv.s = 1.0;
		hsv.v = 1.0;
		pcl::PointXYZRGB rgb;
		pcl::PointXYZHSVtoXYZRGB(hsv, rgb);
		pointcloud[i].r = rgb.r;
		pointcloud[i].g = rgb.g;
		pointcloud[i].b = rgb.b;
	}
}


template <typename PointT>
void colorizePointCloudClusters(const pcl::PointCloud<PointT>& pointcloud, const std::vector<pcl::PointIndices>& cluster_indices, pcl::PointCloud<PointT>& pointcloud_colored_out) {
	for (size_t cluster_index = 0; cluster_index < cluster_indices.size(); ++cluster_index) {
		pcl::RGB cluster_color = pcl::GlasbeyLUT::at(cluster_index % pcl::GlasbeyLUT::size());
		for (size_t point_index = 0; point_index < cluster_indices[cluster_index].indices.size(); ++point_index) {
			PointT point = pointcloud[cluster_indices[cluster_index].indices[point_index]];
			point.r = cluster_color.r;
			point.g = cluster_color.g;
			point.b = cluster_color.b;
			pointcloud_colored_out.push_back(point);
		}
	}
}


template <typename PointT>
void extractPointCloudClusters(const pcl::PointCloud<PointT>& pointcloud, const std::vector<pcl::PointIndices>& cluster_indices, const std::vector<size_t>& selected_clusters, pcl::PointCloud<PointT>& pointcloud_out) {
	for (size_t cluster_index = 0; cluster_index < selected_clusters.size(); ++cluster_index) {
		const std::vector<int>& indices = cluster_indices[selected_clusters[cluster_index]].indices;
		for (size_t point_index = 0; point_index < indices.size(); ++point_index) {
			pointcloud_out.push_back(pointcloud[indices[point_index]]);
		}
	}
}

template <typename PointT>
float distanceSquaredToOrigin(const PointT& point) {
	return point.x * point.x + point.y * point.y + point.z * point.z;
}


} /* namespace pointcloud_utils */
} /* namespace dynamic_robot_localization */
