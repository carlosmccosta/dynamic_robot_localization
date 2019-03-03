/**\file cluster_sorters.hpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/cluster_selectors/cluster_sorters.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


namespace dynamic_robot_localization {
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <ClusterSorters-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
void ClusterSizeSorter<PointT>::sortClusters(const typename pcl::PointCloud<PointT>::Ptr& input_cloud, const std::vector<pcl::PointIndices>& cluster_indices,
		size_t sorted_clusters_subset_start_index, size_t sorted_clusters_subset_end_index, std::vector<size_t>& selected_clusters_out) {
	std::vector< std::pair<size_t, size_t> > clusters;
	for (size_t i = 0; i < cluster_indices.size(); ++i) {
		clusters.push_back(std::pair<size_t, size_t>(i, cluster_indices[i].indices.size()));
	}

	if (ClusterSorter<PointT>::ascending_sort_) {
		std::sort(clusters.begin(), clusters.end(), math_utils::sortFunctionForPairSecondValueAscendingOrder<size_t, size_t>);
	} else {
		std::sort(clusters.begin(), clusters.end(), math_utils::sortFunctionForPairSecondValueDescendingOrder<size_t, size_t>);
	}

	for (size_t i = sorted_clusters_subset_start_index; i < sorted_clusters_subset_end_index; ++i) {
		selected_clusters_out.push_back(clusters[i].first);
	}
}

template<typename PointT>
void DistanceToOriginSorter<PointT>::sortClusters(const typename pcl::PointCloud<PointT>::Ptr& input_cloud, const std::vector<pcl::PointIndices>& cluster_indices,
		size_t sorted_clusters_subset_start_index, size_t sorted_clusters_subset_end_index, std::vector<size_t>& selected_clusters_out) {
	std::vector< std::pair<size_t, float> > clusters;
	for (size_t i = 0; i < cluster_indices.size(); ++i) {
		pcl::PointXYZ sort_point;
		if (ClusterSorter<PointT>::clusters_origins_ && i < ClusterSorter<PointT>::clusters_origins_->size()) {
			sort_point = (*ClusterSorter<PointT>::clusters_origins_)[i];
		} else {
			Eigen::Vector4f centroid;
			pcl::compute3DCentroid(*input_cloud, cluster_indices[i], centroid);
			sort_point.x = centroid(0);
			sort_point.y = centroid(1);
			sort_point.z = centroid(2);
		}

		float distance_to_origin = getDistance(sort_point);
		clusters.push_back(std::pair<size_t, float>(i, distance_to_origin));
	}

	if (ClusterSorter<PointT>::ascending_sort_) {
		std::sort(clusters.begin(), clusters.end(), math_utils::sortFunctionForPairSecondValueAscendingOrder<size_t, float>);
	} else {
		std::sort(clusters.begin(), clusters.end(), math_utils::sortFunctionForPairSecondValueDescendingOrder<size_t, float>);
	}

	for (size_t i = sorted_clusters_subset_start_index; i < sorted_clusters_subset_end_index; ++i) {
		selected_clusters_out.push_back(clusters[i].first);
	}

	ClusterSorter<PointT>::clusters_origins_.reset();
}

template<typename PointT>
float DistanceToOriginSorter<PointT>::getDistance(const pcl::PointXYZ& point) {
	return pointcloud_utils::distanceSquaredToOrigin(point);
}

template<typename PointT>
float AxisValueSorter<PointT>::getDistance(const pcl::PointXYZ& point) {
	float axis_value = point.z;
	if (axis_ == Axis::X) {
		axis_value = point.x;
	} else if (axis_ == Axis::Y) {
		axis_value = point.y;
	}
	return axis_value;
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </ClusterSorters-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

} /* namespace dynamic_robot_localization */
