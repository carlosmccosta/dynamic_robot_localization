/**\file cluster_selector.hpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/cluster_selectors/cluster_selector.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


namespace dynamic_robot_localization {

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <ClusterSelector-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
void ClusterSelector<PointT>::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace) {
	private_node_handle_ = private_node_handle;
	configuration_namespace_ = configuration_namespace;

	loadClustersIndicesFromParameterServer();
	private_node_handle->param(configuration_namespace + "load_clusters_indices_from_parameter_server_before_filtering", load_clusters_indices_from_parameter_server_before_filtering_, true);

	std::string sorting_algorithm;
	private_node_handle->param(configuration_namespace + "sorting_algorithm", sorting_algorithm, std::string("MaxClusterSizeSorter"));

	std::string sorting_axis;
	private_node_handle->param(configuration_namespace + "sorting_axis", sorting_axis, std::string("Z"));
	typename AxisValueSorter<PointT>::Axis axis = AxisValueSorter<PointT>::Axis::Z;
	if (sorting_axis == "X") {
		axis = AxisValueSorter<PointT>::Axis::X;
	} else if (sorting_axis == "Y") {
		axis = AxisValueSorter<PointT>::Axis::Y;
	} else if (sorting_axis == "Z") {
		axis = AxisValueSorter<PointT>::Axis::Z;
	} else {
		ROS_WARN_STREAM("Unsupported sorting axis: [" << sorting_axis << "] -> using default: [Z]");
	}

	if (sorting_algorithm == "MinClusterSizeSorter") {
		cluster_sorter_ = typename ClusterSorter<PointT>::Ptr(new ClusterSizeSorter<PointT>(true));
	} else if (sorting_algorithm == "MaxClusterSizeSorter") {
		cluster_sorter_ = typename ClusterSorter<PointT>::Ptr(new ClusterSizeSorter<PointT>(false));
	} else if (sorting_algorithm == "MinDistanceToOriginSorter") {
		cluster_sorter_ = typename ClusterSorter<PointT>::Ptr(new DistanceToOriginSorter<PointT>(true));
	} else if (sorting_algorithm == "MaxDistanceToOriginSorter") {
		cluster_sorter_ = typename ClusterSorter<PointT>::Ptr(new DistanceToOriginSorter<PointT>(false));
	} else if (sorting_algorithm == "MinAxisValueSorter") {
		cluster_sorter_ = typename ClusterSorter<PointT>::Ptr(new AxisValueSorter<PointT>(true, axis));
	} else if (sorting_algorithm == "MaxAxisValueSorter") {
		cluster_sorter_ = typename ClusterSorter<PointT>::Ptr(new AxisValueSorter<PointT>(false, axis));
	}

	if (!cluster_sorter_) {
		ROS_WARN_STREAM("Unsupported cluster sorter algorithm: [" << sorting_algorithm << "] -> using default: [MaxClusterSizeSorter]");
		cluster_sorter_ = typename ClusterSorter<PointT>::Ptr(new ClusterSizeSorter<PointT>(false));
		selector_name_ += "UsingMaxClusterSizeSorter";
	} else {
		selector_name_ += "Using" + sorting_algorithm;
	}

	clusters_colored_cloud_publisher_ = typename CloudPublisher<PointT>::Ptr(new CloudPublisher<PointT>());
	clusters_colored_cloud_publisher_->setParameterServerArgumentToLoadTopicName(configuration_namespace + "clusters_colored_cloud_publish_topic");
	clusters_colored_cloud_publisher_->setupConfigurationFromParameterServer(node_handle, private_node_handle, configuration_namespace);
}

template<typename PointT>
void ClusterSelector<PointT>::loadClustersIndicesFromParameterServer() {
	std::string search_namespace = private_node_handle_->getNamespace() + "/" + configuration_namespace_;
	std::string final_param_name;
	if (ros::param::search(search_namespace, "min_cluster_index", final_param_name)) { private_node_handle_->param(final_param_name, min_cluster_index_, 0); }
	if (ros::param::search(search_namespace, "max_cluster_index", final_param_name)) { private_node_handle_->param(final_param_name, max_cluster_index_, 1); }
}

template<typename PointT>
void ClusterSelector<PointT>::selectClusters(const typename pcl::PointCloud<PointT>::Ptr& input_cloud, const std::vector<pcl::PointIndices>& cluster_indices, std::vector<size_t>& selected_clusters_out) {
	if (clusters_colored_cloud_publisher_ && !clusters_colored_cloud_publisher_->getCloudPublishTopic().empty()) {
		typename pcl::PointCloud<PointT>::Ptr colored_point_cloud(new pcl::PointCloud<PointT>());
		pointcloud_utils::colorizePointCloudClusters(*input_cloud, cluster_indices, *colored_point_cloud);
		colored_point_cloud->header = input_cloud->header;
		clusters_colored_cloud_publisher_->publishPointCloud(*colored_point_cloud);
	}

	if (load_clusters_indices_from_parameter_server_before_filtering_)
		loadClustersIndicesFromParameterServer();

	size_t start_index = 0;
	if (min_cluster_index_ >= 0 && (size_t)min_cluster_index_ < cluster_indices.size())
		start_index = (size_t)min_cluster_index_;

	size_t end_index = cluster_indices.size();
	if (max_cluster_index_ >= 0 && (size_t)max_cluster_index_ < cluster_indices.size())
		end_index = (size_t)max_cluster_index_;

	cluster_sorter_->sortClusters(input_cloud, cluster_indices, start_index, end_index, selected_clusters_out);

	std::stringstream ss;
	ss << selector_name_ << " selected cluster indices [ ";
	for (size_t i = 0; i < selected_clusters_out.size(); ++i) {
		ss << selected_clusters_out[i];
		if (i < (selected_clusters_out.size() - 1))
			ss << ", ";
	}
	ss << " ]";
	ROS_DEBUG_STREAM(ss.str());
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </ClusterSelector-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

} /* namespace dynamic_robot_localization */
