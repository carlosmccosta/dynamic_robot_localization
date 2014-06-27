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
void FeatureMatcher<PointT, FeatureT>::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle) {
	bool display_feature_matching;
	private_node_handle->param("display_feature_matching", display_feature_matching, false);
	private_node_handle->param("reference_pointcloud_descriptors_filename", reference_pointcloud_descriptors_filename_, std::string(""));
	private_node_handle->param("reference_pointcloud_descriptors_save_filename", reference_pointcloud_descriptors_save_filename_, std::string(""));
	private_node_handle->param("save_descriptors_in_binary_format", save_descriptors_in_binary_format_, true);
	CloudMatcher<PointT>::setDisplayCloudAligment(display_feature_matching);

	keypoint_descriptor_ = typename KeypointDescriptor<PointT, FeatureT>::Ptr(new FPFH<PointT, FeatureT>());
	keypoint_descriptor_->setupConfigurationFromParameterServer(node_handle, private_node_handle);

	CloudMatcher<PointT>::setupConfigurationFromParameterServer(node_handle, private_node_handle);
	CloudMatcher<PointT>::setDisplayCloudAligment(display_feature_matching);
	CloudMatcher<PointT>::setupRegistrationVisualizer();
}


template<typename PointT, typename FeatureT>
void FeatureMatcher<PointT, FeatureT>::setupReferenceCloud(typename pcl::PointCloud<PointT>::Ptr& reference_cloud,
		typename pcl::PointCloud<PointT>::Ptr& reference_cloud_keypoints,
        typename pcl::search::KdTree<PointT>::Ptr& search_method) {

	typename pcl::PointCloud<PointT>::Ptr& reference_cloud_final = reference_cloud_keypoints->points.empty() ? reference_cloud : reference_cloud_keypoints;

	// subclass must set cloud_matcher_ ptr
	if (CloudMatcher<PointT>::getCloudMatcher()) {
		CloudMatcher<PointT>::getCloudMatcher()->setSearchMethodTarget(search_method);
		CloudMatcher<PointT>::getCloudMatcher()->setInputTarget(reference_cloud_final);
	}

	if (CloudMatcher<PointT>::getRegistrationVisualizer()) {
		CloudMatcher<PointT>::getRegistrationVisualizer()->setTargetCloud(*reference_cloud_final);
	}

	typename pcl::PointCloud<FeatureT>::Ptr reference_descriptors(new pcl::PointCloud<FeatureT>());
	if (reference_pointcloud_descriptors_filename_.empty() || pcl::io::loadPCDFile<FeatureT>(reference_pointcloud_descriptors_filename_, *reference_descriptors) != 0) {
		reference_descriptors = keypoint_descriptor_->computeKeypointsDescriptors(reference_cloud_final, reference_cloud, search_method);
	} else {
		ROS_DEBUG_STREAM("Loaded " << reference_descriptors->points.size() << " keypoint descriptors from file " << reference_pointcloud_descriptors_filename_);
	}

	if (!reference_pointcloud_descriptors_save_filename_.empty() && !reference_descriptors->points.empty()) {
		ROS_DEBUG_STREAM("Saving " << reference_descriptors->points.size() << " reference pointcloud keypoint descriptors to file " << reference_pointcloud_descriptors_save_filename_);
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

// =============================================================================   <private-section>   =========================================================================
// =============================================================================   </private-section>  =========================================================================

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <template instantiations>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </template instantiations>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

} /* namespace dynamic_robot_localization */

