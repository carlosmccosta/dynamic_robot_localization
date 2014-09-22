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
	occupancy_grid_analysis_radius_resolution_percentage_(4.0) {}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <NormalEstimator-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
void NormalEstimator<PointT>::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace) {
	std::string final_param_name;
//	if (private_node_handle->searchParam(configuration_namespace + "display_normals", final_param_name)) {
	if (ros::param::search(private_node_handle->getNamespace() + "/" + configuration_namespace, "display_normals", final_param_name)) {
		private_node_handle->param(final_param_name, display_normals_, false);
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
}


template<typename PointT>
void NormalEstimator<PointT>::displayNormals(typename pcl::PointCloud<PointT>::Ptr& pointcloud_with_normals) {
	pcl::visualization::PCLVisualizer normals_visualizer("Normals");
	normals_visualizer.setBackgroundColor (0, 0, 0);
	normals_visualizer.initCameraParameters ();
	normals_visualizer.setCameraPosition(-6, 0, 0, 0, 0, 1);
	normals_visualizer.addCoordinateSystem (0.5, 0);
	normals_visualizer.addPointCloudNormals<PointT>(pointcloud_with_normals, 1, 0.05, VISUALIZER_NORMALS_ID);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler(pointcloud_with_normals, 0, 255, 0);
	normals_visualizer.addPointCloud(pointcloud_with_normals, color_handler, "Cloud points");
	normals_visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Cloud points");

//	normals_visualizer_->spinOnce(5, true);
//	normals_visualizer_->spin();

	while (!normals_visualizer.wasStopped()) {
		normals_visualizer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
//	normals_visualizer.close();
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </NormalEstimator-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// =============================================================================   <private-section>   =========================================================================
// =============================================================================   </private-section>  =========================================================================

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <template instantiations>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </template instantiations>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

} /* namespace dynamic_robot_localization */

