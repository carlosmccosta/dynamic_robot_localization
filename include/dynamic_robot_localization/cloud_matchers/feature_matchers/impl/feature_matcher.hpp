/**\file feature_matcher.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/cloud_matchers/feature_matchers/feature_matcher.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <FeatureMatcher-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT, typename FeatureT>
void FeatureMatcher<PointT, FeatureT>::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace) {
	std::string final_param_name;
	std::string search_namespace = private_node_handle->getNamespace() + "/" + configuration_namespace;
	bool display_feature_matching = false;
	if (ros::param::search(search_namespace, "display_feature_matching", final_param_name)) { private_node_handle->param(final_param_name, display_feature_matching, false); }
	if (ros::param::search(search_namespace, "reference_pointcloud_descriptors_filename", final_param_name)) { private_node_handle->param(final_param_name, reference_pointcloud_descriptors_filename_, std::string("")); }
	if (ros::param::search(search_namespace, "reference_pointcloud_descriptors_save_filename", final_param_name)) { private_node_handle->param(final_param_name, reference_pointcloud_descriptors_save_filename_, std::string("")); }
	if (ros::param::search(search_namespace, "save_descriptors_in_binary_format", final_param_name)) { private_node_handle->param(final_param_name, save_descriptors_in_binary_format_, true); }

	CloudMatcher<PointT>::setDisplayCloudAligment(display_feature_matching);

	CloudMatcher<PointT>::setupConfigurationFromParameterServer(node_handle, private_node_handle, configuration_namespace);
	CloudMatcher<PointT>::setDisplayCloudAligment(display_feature_matching);
	CloudMatcher<PointT>::setupRegistrationVisualizer();
}


template<typename PointT, typename FeatureT>
void FeatureMatcher<PointT, FeatureT>::setupReferenceCloud(typename pcl::PointCloud<PointT>::Ptr& reference_cloud,
		typename pcl::PointCloud<PointT>::Ptr& reference_cloud_keypoints,
		typename pcl::search::KdTree<PointT>::Ptr& search_method) {

	CloudMatcher<PointT>::reference_cloud_ = reference_cloud;
	CloudMatcher<PointT>::reference_cloud_keypoints_ = reference_cloud_keypoints;
	CloudMatcher<PointT>::search_method_ = search_method;

	typename pcl::PointCloud<PointT>::Ptr& reference_cloud_final = reference_cloud_keypoints->empty() ? reference_cloud : reference_cloud_keypoints;

	// subclass must set cloud_matcher_ ptr
	if (CloudMatcher<PointT>::getCloudMatcher()) {
		CloudMatcher<PointT>::getCloudMatcher()->setSearchMethodTarget(search_method, true);
		CloudMatcher<PointT>::getCloudMatcher()->setInputTarget(reference_cloud_final);
	}

	if (CloudMatcher<PointT>::getRegistrationVisualizer()) {
		CloudMatcher<PointT>::getRegistrationVisualizer()->setTargetCloud(*reference_cloud_final);
	}

	typename pcl::PointCloud<FeatureT>::Ptr reference_descriptors(new pcl::PointCloud<FeatureT>());
	if (reference_pointcloud_descriptors_filename_.empty() || !pointcloud_conversions::fromFile(reference_pointcloud_descriptors_filename_, *reference_descriptors)) {
		if (keypoint_descriptor_) // must be set previously
			reference_descriptors = keypoint_descriptor_->computeKeypointsDescriptors(reference_cloud_final, reference_cloud, search_method);
	} else {
		ROS_INFO_STREAM("Loaded " << reference_descriptors->size() << " keypoint descriptors from file " << reference_pointcloud_descriptors_filename_);
	}

	if (!reference_pointcloud_descriptors_save_filename_.empty() && !reference_descriptors->empty()) {
		ROS_INFO_STREAM("Saving " << reference_descriptors->size() << " reference pointcloud keypoint descriptors to file " << reference_pointcloud_descriptors_save_filename_);
		pcl::io::savePCDFile<FeatureT>(reference_pointcloud_descriptors_save_filename_, *reference_descriptors, save_descriptors_in_binary_format_);
	}

	setMatcherReferenceDescriptors(reference_descriptors);
}


template<typename PointT, typename FeatureT>
void FeatureMatcher<PointT, FeatureT>::initializeKeypointProcessing() {
	CloudMatcher<PointT>::setMatchOnlyKeypoints(true);
}


template<typename PointT, typename FeatureT>
void FeatureMatcher<PointT, FeatureT>::processKeypoints(typename pcl::PointCloud<PointT>::Ptr& pointcloud_keypoints,
		typename pcl::PointCloud<PointT>::Ptr& surface,
		typename pcl::search::KdTree<PointT>::Ptr& surface_search_method) {

	typename pcl::PointCloud<FeatureT>::Ptr ambient_descriptors = keypoint_descriptor_->computeKeypointsDescriptors(pointcloud_keypoints, surface, surface_search_method);
	setMatcherAmbientDescriptors(ambient_descriptors);
	CloudMatcher<PointT>::setMatchOnlyKeypoints(true);
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </FeatureMatcher-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <template instantiations>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </template instantiations>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

} /* namespace dynamic_robot_localization */

