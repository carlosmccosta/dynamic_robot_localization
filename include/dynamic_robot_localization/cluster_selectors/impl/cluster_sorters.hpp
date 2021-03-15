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
void ClusterSorter<PointT>::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, const std::string& configuration_namespace) {
	private_node_handle_ = private_node_handle;
	configuration_namespace_ = configuration_namespace;

	transform_from_cloud_frame_to_sorting_tf_frame_.setIdentity();
	loadTfNameForSortigClusters();
	private_node_handle->param(configuration_namespace + "load_tf_name_for_sorting_clusters_before_processing", load_tf_name_for_sorting_clusters_before_processing_, true);
	private_node_handle->param(configuration_namespace + "use_cloud_time_for_retrieving_tf_transform", use_cloud_time_for_retrieving_tf_transform_, true);

	double tf_timeout;
	private_node_handle_->param(configuration_namespace + "message_management/tf_timeout", tf_timeout, 0.5);
	tf_timeout_ = ros::Duration(tf_timeout);
}

template<typename PointT>
void ClusterSorter<PointT>::loadTfNameForSortigClusters() {
	std::string search_namespace = private_node_handle_->getNamespace() + "/" + configuration_namespace_;
	std::string final_param_name;
	if (ros::param::search(search_namespace, "tf_name_for_sorting_clusters", final_param_name)) { private_node_handle_->param(final_param_name, tf_name_for_sorting_clusters_, std::string("")); }
}

template<typename PointT>
bool ClusterSorter<PointT>::retrieveTfForSortigClusters(const std::string& cloud_frame_id, const ros::Time& cloud_time) {
	if (ClusterSorter<PointT>::tf_collector_ != nullptr && !ClusterSorter<PointT>::tf_name_for_sorting_clusters_.empty()) {
		bool tf_available = false;
		if (use_cloud_time_for_retrieving_tf_transform_) {
			tf_available = ClusterSorter<PointT>::tf_collector_->lookForTransform(transform_from_cloud_frame_to_sorting_tf_frame_, cloud_frame_id, tf_name_for_sorting_clusters_, cloud_time, tf_timeout_);
		} else {
			tf_available = ClusterSorter<PointT>::tf_collector_->lookForTransform(transform_from_cloud_frame_to_sorting_tf_frame_, cloud_frame_id, tf_name_for_sorting_clusters_, ros::Time(0), tf_timeout_);
		}

		if (!tf_available) {
			transform_from_cloud_frame_to_sorting_tf_frame_.setIdentity();
		}
		return tf_available;
	} else {
		transform_from_cloud_frame_to_sorting_tf_frame_.setIdentity();
		return true;
	}
}

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
	if (ClusterSorter<PointT>::load_tf_name_for_sorting_clusters_before_processing_) {
		ClusterSorter<PointT>::loadTfNameForSortigClusters();
	}

	ros::Time cloud_time = pcl_conversions::fromPCL(input_cloud->header.stamp);
	ClusterSorter<PointT>::retrieveTfForSortigClusters(input_cloud->header.frame_id, cloud_time);
	pcl::PointXYZ tf_sorting_point;
	tf_sorting_point.x = ClusterSorter<PointT>::transform_from_cloud_frame_to_sorting_tf_frame_.getOrigin().getX();
	tf_sorting_point.y = ClusterSorter<PointT>::transform_from_cloud_frame_to_sorting_tf_frame_.getOrigin().getY();
	tf_sorting_point.z = ClusterSorter<PointT>::transform_from_cloud_frame_to_sorting_tf_frame_.getOrigin().getZ();

	ROS_DEBUG_STREAM("[ClusterSorter] tf_sorting_point: [ " << tf_sorting_point.x << " | " << tf_sorting_point.y << " | " << tf_sorting_point.z << " ]");

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
		sort_point.x -= tf_sorting_point.x;
		sort_point.y -= tf_sorting_point.y;
		sort_point.z -= tf_sorting_point.z;

		float distance_squared_to_origin = getDistanceSquared(sort_point);
		clusters.push_back(std::pair<size_t, float>(i, distance_squared_to_origin));
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
float DistanceToOriginSorter<PointT>::getDistanceSquared(const pcl::PointXYZ& point) {
	return pointcloud_utils::distanceSquaredToOrigin(point);
}

template<typename PointT>
float AxisValueSorter<PointT>::getDistanceSquared(const pcl::PointXYZ& point) {
	float axis_value = point.z * point.z;
	if (axis_ == Axis::X) {
		axis_value = point.x * point.x;
	} else if (axis_ == Axis::Y) {
		axis_value = point.y * point.y;
	}
	return axis_value;
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </ClusterSorters-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

} /* namespace dynamic_robot_localization */
