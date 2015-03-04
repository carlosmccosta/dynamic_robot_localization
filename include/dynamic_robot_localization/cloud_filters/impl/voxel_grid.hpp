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
void VoxelGrid<PointT>::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace) {
	typename pcl::Filter<PointT>::Ptr filter_base(new pcl::VoxelGrid<PointT>());
	typename boost::shared_ptr< pcl::VoxelGrid<PointT> > filter = boost::static_pointer_cast< typename pcl::VoxelGrid<PointT> >(filter_base);

	double leaf_size_x, leaf_size_y, leaf_size_z;
	private_node_handle->param(configuration_namespace + "leaf_size_x", leaf_size_x, 0.01);
	private_node_handle->param(configuration_namespace + "leaf_size_y", leaf_size_y, 0.01);
	private_node_handle->param(configuration_namespace + "leaf_size_z", leaf_size_z, 0.01);
	filter->setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);

	bool downsample_all_data;
	private_node_handle->param(configuration_namespace + "downsample_all_data", downsample_all_data, false);
	filter->setDownsampleAllData(downsample_all_data);

	bool save_leaf_layout;
	private_node_handle->param(configuration_namespace + "save_leaf_layout", save_leaf_layout, false);
	filter->setSaveLeafLayout(save_leaf_layout);

	// only in latest pcl devel branch
	/*int min_points_per_voxel;
	private_node_handle->param("min_points_per_voxel", min_points_per_voxel, 3);
	filter_.setMinimumPointsNumberPerVoxel((unsigned int)min_points_per_voxel);*/

	std::string filter_limit_field_name;
	private_node_handle->param(configuration_namespace + "filter_limit_field_name", filter_limit_field_name, std::string("z"));
	filter->setFilterFieldName(filter_limit_field_name);

	double filter_limit_min, filter_limit_max;
	private_node_handle->param(configuration_namespace + "filter_limit_min", filter_limit_min, -5.0);
	private_node_handle->param(configuration_namespace + "filter_limit_max", filter_limit_max, 5.0);
	filter->setFilterLimits(filter_limit_min, filter_limit_max);

	CloudFilter<PointT>::setFilter(filter_base);
	CloudFilter<PointT>::setupConfigurationFromParameterServer(node_handle, private_node_handle, configuration_namespace);
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </VoxelFilter-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

} /* namespace dynamic_robot_localization */
