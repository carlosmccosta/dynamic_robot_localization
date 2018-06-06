/**\file cloud_viewer.hpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/common/cloud_viewer.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <CloudPublisher-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
CloudViewer<PointT>::CloudViewer() :
		viewer_camera_px_(0.0),
		viewer_camera_py_(0.0),
		viewer_camera_pz_(2.0),
		viewer_camera_up_x_(0.0),
		viewer_camera_up_y_(-1.0),
		viewer_camera_up_z_(0.0),
		viewer_normals_size_(0.01),
		viewer_axis_size_(0.2),
		viewer_background_r_(0.0),
		viewer_background_g_(0.0),
		viewer_background_b_(0.0)
	{}


template<typename PointT>
void CloudViewer<PointT>::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace) {
	std::string final_param_name;

	if (ros::param::search(private_node_handle->getNamespace() + "/" + configuration_namespace, "camera_px", final_param_name)) {
		private_node_handle->param(final_param_name, viewer_camera_px_, 0.0);
	}

	if (ros::param::search(private_node_handle->getNamespace() + "/" + configuration_namespace, "camera_py", final_param_name)) {
		private_node_handle->param(final_param_name, viewer_camera_py_, 0.0);
	}

	if (ros::param::search(private_node_handle->getNamespace() + "/" + configuration_namespace, "camera_pz", final_param_name)) {
		private_node_handle->param(final_param_name, viewer_camera_pz_, 2.0);
	}

	if (ros::param::search(private_node_handle->getNamespace() + "/" + configuration_namespace, "camera_up_x", final_param_name)) {
		private_node_handle->param(final_param_name, viewer_camera_up_x_, 0.0);
	}

	if (ros::param::search(private_node_handle->getNamespace() + "/" + configuration_namespace, "camera_up_y", final_param_name)) {
		private_node_handle->param(final_param_name, viewer_camera_up_y_, -1.0);
	}

	if (ros::param::search(private_node_handle->getNamespace() + "/" + configuration_namespace, "camera_up_z", final_param_name)) {
		private_node_handle->param(final_param_name, viewer_camera_up_z_, 0.0);
	}

	if (ros::param::search(private_node_handle->getNamespace() + "/" + configuration_namespace, "normals_size", final_param_name)) {
		private_node_handle->param(final_param_name, viewer_normals_size_, 0.01);
	}

	if (ros::param::search(private_node_handle->getNamespace() + "/" + configuration_namespace, "axis_size", final_param_name)) {
		private_node_handle->param(final_param_name, viewer_axis_size_, 0.2);
	}

	if (ros::param::search(private_node_handle->getNamespace() + "/" + configuration_namespace, "background_r", final_param_name)) {
		private_node_handle->param(final_param_name, viewer_background_r_, 0.0);
	}

	if (ros::param::search(private_node_handle->getNamespace() + "/" + configuration_namespace, "background_g", final_param_name)) {
		private_node_handle->param(final_param_name, viewer_background_g_, 0.0);
	}

	if (ros::param::search(private_node_handle->getNamespace() + "/" + configuration_namespace, "background_b", final_param_name)) {
		private_node_handle->param(final_param_name, viewer_background_b_, 0.0);
	}
}


template<typename PointT>
void CloudViewer<PointT>::showPointCloud(typename pcl::PointCloud<PointT>::Ptr& pointcloud_with_normals, std::string window_name) {
	pcl::visualization::PCLVisualizer cloud_visualizer(window_name);
	cloud_visualizer.setBackgroundColor (viewer_background_r_, viewer_background_g_, viewer_background_b_);
	cloud_visualizer.initCameraParameters ();
	cloud_visualizer.addCoordinateSystem (viewer_axis_size_);
	cloud_visualizer.addPointCloudNormals<PointT>(pointcloud_with_normals, 1, viewer_normals_size_, "normals");
	pcl::visualization::PointCloudColorHandlerGenericField<PointT> color_handler(pointcloud_with_normals, "curvature");
	cloud_visualizer.addPointCloud(pointcloud_with_normals, color_handler, "Cloud points");
	cloud_visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Cloud points");
	cloud_visualizer.setCameraPosition(viewer_camera_px_, viewer_camera_py_, viewer_camera_pz_,
										 viewer_camera_up_x_, viewer_camera_up_y_, viewer_camera_up_z_);

//	normals_visualizer_->spinOnce(5, true);
//	normals_visualizer_->spin();

	while (!cloud_visualizer.wasStopped()) {
		cloud_visualizer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	cloud_visualizer.close();
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </CloudPublisher-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

} /* namespace dynamic_robot_localization */
