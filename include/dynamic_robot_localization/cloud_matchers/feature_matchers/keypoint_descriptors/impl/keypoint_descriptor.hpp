/**\file keypoint_descriptor.hpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/cloud_matchers/feature_matchers/keypoint_descriptors/keypoint_descriptor.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <KeypointDescriptor-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT, typename FeatureT>
void KeypointDescriptor<PointT, FeatureT>::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace) {
	if (feature_descriptor_) { // subclass must set pointer first
		std::string final_param_name;
		std::string search_namespace = private_node_handle->getNamespace() + "/" + configuration_namespace;

		int feature_descriptor_k_search = 10;
		if (ros::param::search(search_namespace, "feature_descriptor_k_search", final_param_name)) { private_node_handle->param(final_param_name, feature_descriptor_k_search, 10); }
		feature_descriptor_->setKSearch(feature_descriptor_k_search);

		if (feature_descriptor_k_search <= 0) {
			double feature_descriptor_radius_search = 0.1;
			if (ros::param::search(search_namespace, "feature_descriptor_radius_search", final_param_name)) { private_node_handle->param(final_param_name, feature_descriptor_radius_search, 0.1); }
			feature_descriptor_->setRadiusSearch(feature_descriptor_radius_search);
		} else {
			feature_descriptor_->setRadiusSearch(0.0);
		}
	}
}


template<typename PointT, typename FeatureT>
typename pcl::PointCloud<FeatureT>::Ptr KeypointDescriptor<PointT, FeatureT>::computeKeypointsDescriptors(
        typename pcl::PointCloud<PointT>::Ptr& pointcloud_keypoints, typename pcl::PointCloud<PointT>::Ptr& surface,
        typename pcl::search::KdTree<PointT>::Ptr& surface_search_method) {

	typename pcl::PointCloud<FeatureT>::Ptr descriptors(new pcl::PointCloud<FeatureT>());

	if (feature_descriptor_) {
		ROS_DEBUG_STREAM("Computing descriptors for " << pointcloud_keypoints->size() << " keypoints");
		feature_descriptor_->setSearchMethod(surface_search_method);
		feature_descriptor_->setSearchSurface(surface);
		feature_descriptor_->setInputCloud(pointcloud_keypoints);
		feature_descriptor_->compute(*descriptors);
		ROS_DEBUG_STREAM("Finished computation of " << pointcloud_keypoints->size() << " descriptors");
	}

	return descriptors;
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </KeypointDescriptor-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <template instantiations>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </template instantiations>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

} /* namespace dynamic_robot_localization */
