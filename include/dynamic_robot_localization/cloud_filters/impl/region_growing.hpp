/**\file region_growing.hpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/cloud_filters/region_growing.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <RegionGrowing-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
void RegionGrowing<PointT>::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace) {
	bool use_pointcloud_rgb_information;
	private_node_handle->param(configuration_namespace + "use_pointcloud_rgb_information", use_pointcloud_rgb_information, false);

	if (use_pointcloud_rgb_information) {
		typename std::shared_ptr< pcl::RegionGrowingRGB<PointT, PointT> > region_growing_rgb(new pcl::RegionGrowingRGB<PointT, PointT>());

		double point_color_threshold;
		private_node_handle->param(configuration_namespace + "point_color_threshold", point_color_threshold, 1200.0);
		region_growing_rgb->setPointColorThreshold(point_color_threshold);

		double region_color_threshold;
		private_node_handle->param(configuration_namespace + "region_color_threshold", region_color_threshold, 1200.0);
		region_growing_rgb->setRegionColorThreshold(region_color_threshold);

		double distance_threshold;
		private_node_handle->param(configuration_namespace + "distance_threshold", distance_threshold, 0.01);
		region_growing_rgb->setDistanceThreshold(distance_threshold);

		int number_of_region_neighbors;
		private_node_handle->param(configuration_namespace + "number_of_region_neighbors", number_of_region_neighbors, 50);
		region_growing_rgb->setNumberOfRegionNeighbours(number_of_region_neighbors);

		bool use_normal_test;
		private_node_handle->param(configuration_namespace + "use_normal_test", use_normal_test, false);
		region_growing_rgb->setNormalTestFlag(use_normal_test);

		region_growing_ = region_growing_rgb;
	} else {
		region_growing_ = typename std::shared_ptr< pcl::RegionGrowing<PointT, PointT> > (new pcl::RegionGrowing<PointT, PointT>());
	}

	int min_cluster_size, max_cluster_size;
	private_node_handle->param(configuration_namespace + "min_cluster_size", min_cluster_size, 100);
	private_node_handle->param(configuration_namespace + "max_cluster_size", max_cluster_size, std::numeric_limits<int>::max());
	region_growing_->setMinClusterSize(min_cluster_size);
	region_growing_->setMaxClusterSize(max_cluster_size);

	bool use_smoothness_constraint;
	private_node_handle->param(configuration_namespace + "use_smoothness_constraint", use_smoothness_constraint, true);
	region_growing_->setSmoothModeFlag(use_smoothness_constraint);

	bool use_curvature_test;
	if (use_pointcloud_rgb_information)
		private_node_handle->param(configuration_namespace + "use_curvature_test", use_curvature_test, false);
	else
		private_node_handle->param(configuration_namespace + "use_curvature_test", use_curvature_test, true);
	region_growing_->setCurvatureTestFlag(use_curvature_test);

	bool use_residual_test;
	private_node_handle->param(configuration_namespace + "use_residual_test", use_residual_test, true);
	region_growing_->setResidualTestFlag(use_residual_test);

	double smoothness_threshold_in_degrees;
	private_node_handle->param(configuration_namespace + "smoothness_threshold_in_degrees", smoothness_threshold_in_degrees, 70.0);
	region_growing_->setSmoothnessThreshold(smoothness_threshold_in_degrees / (180.0 * M_PI));

	double residual_threshold_in_degrees;
	private_node_handle->param(configuration_namespace + "residual_threshold_in_degrees", residual_threshold_in_degrees, 10.0);
	region_growing_->setResidualThreshold(std::cos(residual_threshold_in_degrees / (180.0 * M_PI)));

	double curvature_threshold;
	private_node_handle->param(configuration_namespace + "curvature_threshold", curvature_threshold, 0.2);
	region_growing_->setCurvatureThreshold(curvature_threshold);

	int number_of_neighbors;
	private_node_handle->param(configuration_namespace + "number_of_neighbors", number_of_neighbors, 50);
	region_growing_->setNumberOfNeighbours(number_of_neighbors);

	cluster_selector_.setupConfigurationFromParameterServer(node_handle, private_node_handle, configuration_namespace + "cluster_selector/");
	CloudFilter<PointT>::setupConfigurationFromParameterServer(node_handle, private_node_handle, configuration_namespace);
}


template<typename PointT>
void RegionGrowing<PointT>::filter(const typename pcl::PointCloud<PointT>::Ptr& input_cloud, typename pcl::PointCloud<PointT>::Ptr& output_cloud) {
	size_t number_of_points_in_input_cloud = input_cloud->size();

	typename pcl::search::KdTree<PointT>::Ptr search_tree(new pcl::search::KdTree<PointT>());
	search_tree->setInputCloud(input_cloud);
	region_growing_->setSearchMethod(search_tree);
	region_growing_->setInputCloud(input_cloud);
	region_growing_->setInputNormals(input_cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	region_growing_->extract(cluster_indices);

	std::vector<size_t> selected_clusters;
	cluster_selector_.selectClusters(input_cloud, cluster_indices, selected_clusters);
	pointcloud_utils::extractPointCloudClusters(*input_cloud, cluster_indices, selected_clusters, *output_cloud);

	if (CloudFilter<PointT>::getCloudPublisher() && output_cloud) { CloudFilter<PointT>::getCloudPublisher()->publishPointCloud(*output_cloud); }
	ROS_DEBUG_STREAM(CloudFilter<PointT>::filter_name_ << " filter found " << cluster_indices.size() << " clusters and reduced point cloud from " << number_of_points_in_input_cloud << " points to " << output_cloud->size() << " points");
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </RegionGrowing-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

} /* namespace dynamic_robot_localization */
