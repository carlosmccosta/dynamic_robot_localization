/**\file voxel_filter.hpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/cloud_filters/voxel_grid.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <VoxelFilter-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
void VoxelGrid<PointT>::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle) {
	double leaf_size_x, leaf_size_y, leaf_size_z;
	private_node_handle->param("leaf_size_x", leaf_size_x, 0.01);
	private_node_handle->param("leaf_size_y", leaf_size_y, 0.01);
	private_node_handle->param("leaf_size_z", leaf_size_z, 0.01);
	filter_.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);

	bool downsample_all_data;
	private_node_handle->param("downsample_all_data", downsample_all_data, false);
	filter_.setDownsampleAllData(downsample_all_data);

	bool save_leaf_layout;
	private_node_handle->param("save_leaf_layout", save_leaf_layout, false);
	filter_.setSaveLeafLayout(save_leaf_layout);

	// only in latest pcl devel branch
	/*int min_points_per_voxel;
	private_node_handle->param("min_points_per_voxel", min_points_per_voxel, 3);
	filter_.setMinimumPointsNumberPerVoxel((unsigned int)min_points_per_voxel);*/

	std::string filter_limit_field_name;
	private_node_handle->param("filter_limit_field_name", filter_limit_field_name, std::string("z"));
	filter_.setFilterFieldName(filter_limit_field_name);

	double filter_limit_min, filter_limit_max;
	private_node_handle->param("filter_limit_min", filter_limit_min, -1.0);
	private_node_handle->param("filter_limit_max", filter_limit_max, 3.0);
	filter_.setFilterLimits(filter_limit_min, filter_limit_max);

	typename CloudPublisher<PointT>::Ptr cloud_publisher(new CloudPublisher<PointT>());
	cloud_publisher->setParameterServerArgumentToLoadTopicName("voxel_grid_filtered_cloud_publish_topic");
	cloud_publisher->setupConfigurationFromParameterServer(node_handle, private_node_handle);
	CloudFilter<PointT>::setCloudPublisher(cloud_publisher);
}

template<typename PointT>
void VoxelGrid<PointT>::filter(const typename pcl::PointCloud<PointT>::Ptr& input_cloud, typename pcl::PointCloud<PointT>::Ptr& output_cloud) {
	size_t number_of_points_in_input_cloud = input_cloud->points.size();
	filter_.setInputCloud(input_cloud);
	filter_.filter(*output_cloud);

	CloudFilter<PointT>::getCloudPublisher()->publishPointCloud(*output_cloud);
	ROS_DEBUG_STREAM("VoxelGrid reduced point cloud from " << number_of_points_in_input_cloud << " points to " << output_cloud->points.size() << " points");
}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </VoxelFilter-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// =============================================================================   <private-section>   =========================================================================
// =============================================================================   </private-section>  =========================================================================

} /* namespace dynamic_robot_localization */
