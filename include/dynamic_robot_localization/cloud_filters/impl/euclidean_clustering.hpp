/**\file euclidean_clustering.hpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/cloud_filters/euclidean_clustering.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <EuclideanClustering-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
void EuclideanClustering<PointT>::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace) {
	private_node_handle_ = private_node_handle;
	configuration_namespace_ = configuration_namespace;

	double cluster_tolerance;
	int min_cluster_size, max_cluster_size;
	private_node_handle->param(configuration_namespace + "cluster_tolerance", cluster_tolerance, 0.02);
	private_node_handle->param(configuration_namespace + "min_cluster_size", min_cluster_size, 25);
	private_node_handle->param(configuration_namespace + "max_cluster_size", max_cluster_size, std::numeric_limits<int>::max());
	loadClustersIndicesFromParameterServer();

	private_node_handle->param(configuration_namespace + "load_clusters_indices_from_parameter_server_before_filtering", load_clusters_indices_from_parameter_server_before_filtering_, true);

	euclidean_cluster_extraction_.setClusterTolerance(cluster_tolerance);
	euclidean_cluster_extraction_.setMinClusterSize(min_cluster_size);
	euclidean_cluster_extraction_.setMaxClusterSize(max_cluster_size);

	clusters_colored_cloud_publisher_ = typename CloudPublisher<PointT>::Ptr(new CloudPublisher<PointT>());
	clusters_colored_cloud_publisher_->setParameterServerArgumentToLoadTopicName(configuration_namespace + "clusters_colored_cloud_publish_topic");
	clusters_colored_cloud_publisher_->setupConfigurationFromParameterServer(node_handle, private_node_handle, configuration_namespace);

	CloudFilter<PointT>::setupConfigurationFromParameterServer(node_handle, private_node_handle, configuration_namespace);
}


template<typename PointT>
void EuclideanClustering<PointT>::loadClustersIndicesFromParameterServer() {
	private_node_handle_->param(configuration_namespace_ + "min_cluster_index", min_cluster_index_, 0);
	private_node_handle_->param(configuration_namespace_ + "max_cluster_index", max_cluster_index_, 1);
}


template<typename PointT>
void EuclideanClustering<PointT>::filter(const typename pcl::PointCloud<PointT>::Ptr& input_cloud, typename pcl::PointCloud<PointT>::Ptr& output_cloud) {
	size_t number_of_points_in_input_cloud = input_cloud->size();

	if (load_clusters_indices_from_parameter_server_before_filtering_)
		loadClustersIndicesFromParameterServer();

	typename pcl::search::KdTree<PointT>::Ptr search_tree(new pcl::search::KdTree<PointT>());
	search_tree->setInputCloud(input_cloud);
	euclidean_cluster_extraction_.setSearchMethod(search_tree);
	euclidean_cluster_extraction_.setInputCloud(input_cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	euclidean_cluster_extraction_.extract(cluster_indices);

	size_t start_index = 0;
	if (min_cluster_index_ >= 0 && (size_t)min_cluster_index_ < cluster_indices.size())
		start_index = (size_t)min_cluster_index_;

	size_t end_index = cluster_indices.size();
	if (max_cluster_index_ >= 0 && (size_t)max_cluster_index_ < cluster_indices.size())
		end_index = (size_t)max_cluster_index_;

	typename pcl::PointCloud<PointT>::Ptr colored_point_cloud(new pcl::PointCloud<PointT>());
	colored_point_cloud->header = input_cloud->header;
	bool publish_colored_point_cloud = clusters_colored_cloud_publisher_ && !clusters_colored_cloud_publisher_->getCloudPublishTopic().empty();

	for (size_t cluster_index = 0; cluster_index < cluster_indices.size(); ++cluster_index) {
		pcl::RGB cluster_color = pcl::GlasbeyLUT::at(cluster_index % pcl::GlasbeyLUT::size());
		for (size_t point_index = 0; point_index < cluster_indices[cluster_index].indices.size(); ++point_index) {
			PointT point = input_cloud->points[cluster_indices[cluster_index].indices[point_index]];

			if (cluster_index >= start_index && cluster_index < end_index) {
				output_cloud->push_back(point);
			}

			if (publish_colored_point_cloud) {
				point.r = cluster_color.r;
				point.g = cluster_color.g;
				point.b = cluster_color.b;
				colored_point_cloud->push_back(point);
			}
		}
	}

	if (publish_colored_point_cloud)
		clusters_colored_cloud_publisher_->publishPointCloud(*colored_point_cloud);

	if (CloudFilter<PointT>::getCloudPublisher() && output_cloud) { CloudFilter<PointT>::getCloudPublisher()->publishPointCloud(*output_cloud); }
	ROS_DEBUG_STREAM(CloudFilter<PointT>::filter_name_ << " filter reduced point cloud from " << number_of_points_in_input_cloud << " points to " << output_cloud->size() << " points");
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </EuclideanClustering-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

} /* namespace dynamic_robot_localization */
