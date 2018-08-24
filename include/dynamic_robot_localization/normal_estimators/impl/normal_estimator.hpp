/**\file normal_estimator.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/normal_estimators/normal_estimator.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


namespace dynamic_robot_localization {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
NormalEstimator<PointT>::NormalEstimator() :
	display_normals_(false),
	display_occupancy_grid_pointcloud_(false),
	occupancy_grid_analysis_k_(0),
	occupancy_grid_analysis_radius_(-1.0),
	occupancy_grid_analysis_radius_resolution_percentage_(4.0),
	flip_normals_towards_custom_viewpoint_(false),
	normals_viewpoint_px_(0.0f),
	normals_viewpoint_py_(0.0f),
	normals_viewpoint_pz_(0.0f),
	normalize_normals_(true) {}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <NormalEstimator-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
void NormalEstimator<PointT>::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace) {
	std::string final_param_name;
//	if (private_node_handle->searchParam(configuration_namespace + "display_normals", final_param_name)) {
	if (ros::param::search(private_node_handle->getNamespace() + "/" + configuration_namespace, "display_normals", final_param_name)) {
		private_node_handle->param(final_param_name, display_normals_, false);
	}

	if (display_normals_) {
		cloud_viewer_.setupConfigurationFromParameterServer(node_handle, private_node_handle, configuration_namespace + "normals_viewer/");
	}

	if (ros::param::search(private_node_handle->getNamespace() + "/" + configuration_namespace, "display_occupancy_grid_pointcloud", final_param_name)) {
		private_node_handle->param(final_param_name, display_occupancy_grid_pointcloud_, false);
	}

	if (ros::param::search(private_node_handle->getNamespace() + "/" + configuration_namespace, "occupancy_grid_analysis_k", final_param_name)) {
		private_node_handle->param(final_param_name, occupancy_grid_analysis_k_, 0);
	}

	if (ros::param::search(private_node_handle->getNamespace() + "/" + configuration_namespace, "occupancy_grid_analysis_radius", final_param_name)) {
		private_node_handle->param(final_param_name, occupancy_grid_analysis_radius_, -1.0);
	}

	if (ros::param::search(private_node_handle->getNamespace() + "/" + configuration_namespace, "occupancy_grid_analysis_radius_resolution_percentage", final_param_name)) {
		private_node_handle->param(final_param_name, occupancy_grid_analysis_radius_resolution_percentage_, 4.0);
	}

	if (ros::param::search(private_node_handle->getNamespace() + "/" + configuration_namespace, "flip_normals_towards_custom_viewpoint", final_param_name)) {
		private_node_handle->param(final_param_name, flip_normals_towards_custom_viewpoint_, false);
	}

	if (ros::param::search(private_node_handle->getNamespace() + "/" + configuration_namespace, "normals_viewpoint/px", final_param_name)) {
		private_node_handle->param(final_param_name, normals_viewpoint_px_, 0.0f);
	}

	if (ros::param::search(private_node_handle->getNamespace() + "/" + configuration_namespace, "normals_viewpoint/py", final_param_name)) {
		private_node_handle->param(final_param_name, normals_viewpoint_py_, 0.0f);
	}

	if (ros::param::search(private_node_handle->getNamespace() + "/" + configuration_namespace, "normals_viewpoint/pz", final_param_name)) {
		private_node_handle->param(final_param_name, normals_viewpoint_pz_, 0.0f);
	}

	if (ros::param::search(private_node_handle->getNamespace() + "/" + configuration_namespace, "normalize_normals", final_param_name)) {
		private_node_handle->param(final_param_name, normalize_normals_, true);
	}
}


template<typename PointT>
void NormalEstimator<PointT>::estimateNormals(typename pcl::PointCloud<PointT>::Ptr& pointcloud,
		typename pcl::PointCloud<PointT>::Ptr& surface, typename pcl::search::KdTree<PointT>::Ptr& surface_search_method, tf2::Transform& viewpoint_guess,
		typename pcl::PointCloud<PointT>::Ptr& pointcloud_with_normals_out) {
	if (flip_normals_towards_custom_viewpoint_) {
		for (size_t i = 0; i < pointcloud_with_normals_out->size(); ++i) {
			pcl::flipNormalTowardsViewpoint((*pointcloud_with_normals_out)[i], normals_viewpoint_px_, normals_viewpoint_py_, normals_viewpoint_pz_,
											(*pointcloud_with_normals_out)[i].normal_x, (*pointcloud_with_normals_out)[i].normal_y, (*pointcloud_with_normals_out)[i].normal_z);
		}
	}

	if (normalize_normals_) {
		for (size_t i = 0; i < pointcloud_with_normals_out->size(); ++i) {
			(*pointcloud_with_normals_out)[i].getNormalVector3fMap().normalize();
		}
	}

	if (display_normals_) {
		displayNormals(pointcloud_with_normals_out);
	}
}


template<typename PointT>
void NormalEstimator<PointT>::displayNormals(typename pcl::PointCloud<PointT>::Ptr& pointcloud_with_normals) {
	cloud_viewer_.showPointCloud(pointcloud_with_normals, "Point cloud normals");
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </NormalEstimator-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <template instantiations>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </template instantiations>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

} /* namespace dynamic_robot_localization */

