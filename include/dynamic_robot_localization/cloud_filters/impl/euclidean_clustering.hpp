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
	double cluster_tolerance;
	int min_cluster_size, max_cluster_size;
	private_node_handle->param(configuration_namespace + "cluster_tolerance", cluster_tolerance, 0.02);
	private_node_handle->param(configuration_namespace + "min_cluster_size", min_cluster_size, 25);
	private_node_handle->param(configuration_namespace + "max_cluster_size", max_cluster_size, std::numeric_limits<int>::max());

	euclidean_cluster_extraction_.setClusterTolerance(cluster_tolerance);
	euclidean_cluster_extraction_.setMinClusterSize(min_cluster_size);
	euclidean_cluster_extraction_.setMaxClusterSize(max_cluster_size);

	cluster_selector_.setupConfigurationFromParameterServer(node_handle, private_node_handle, configuration_namespace + "cluster_selector/");
	CloudFilter<PointT>::setupConfigurationFromParameterServer(node_handle, private_node_handle, configuration_namespace);
}


template<typename PointT>
void EuclideanClustering<PointT>::filter(const typename pcl::PointCloud<PointT>::Ptr& input_cloud, typename pcl::PointCloud<PointT>::Ptr& output_cloud) {
	size_t number_of_points_in_input_cloud = input_cloud->size();

	typename pcl::search::KdTree<PointT>::Ptr search_tree(new pcl::search::KdTree<PointT>());
	search_tree->setInputCloud(input_cloud);
	euclidean_cluster_extraction_.setSearchMethod(search_tree);
	euclidean_cluster_extraction_.setInputCloud(input_cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	euclidean_cluster_extraction_.extract(cluster_indices);

	std::vector<size_t> selected_clusters;
	cluster_selector_.selectClusters(input_cloud, cluster_indices, selected_clusters);
	pointcloud_utils::extractPointCloudClusters(*input_cloud, cluster_indices, selected_clusters, *output_cloud);

	if (CloudFilter<PointT>::getCloudPublisher() && output_cloud) { CloudFilter<PointT>::getCloudPublisher()->publishPointCloud(*output_cloud); }
	ROS_DEBUG_STREAM(CloudFilter<PointT>::filter_name_ << " filter found " << cluster_indices.size() << " clusters and reduced point cloud from " << number_of_points_in_input_cloud << " points to " << output_cloud->size() << " points");
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </EuclideanClustering-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

} /* namespace dynamic_robot_localization */
