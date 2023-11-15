/**\file keypoint_detector.pp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/cloud_matchers/feature_matchers/keypoint_detectors/keypoint_detector.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <KeypointDetector-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
void KeypointDetector<PointT>::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, const std::string& configuration_namespace) {
	std::string final_param_name;
	std::string search_namespace = private_node_handle->getNamespace() + "/" + configuration_namespace;
	if (ros::param::search(search_namespace, "retrieve_original_points_metadata_for_detected_keypoints", final_param_name)) { private_node_handle->param(final_param_name, retrieve_original_points_metadata_for_detected_keypoints_, true); }
}


template<typename PointT>
void KeypointDetector<PointT>::findKeypoints(typename pcl::PointCloud<PointT>::Ptr& pointcloud, typename pcl::PointCloud<PointT>::Ptr& pointcloud_keypoints_out,
		typename pcl::PointCloud<PointT>::Ptr& surface, typename pcl::search::KdTree<PointT>::Ptr& surface_search_method) {

	ROS_DEBUG_STREAM("Finding keypoints in a pointcloud with " << pointcloud->size());
	keypoint_detector_->setSearchMethod(surface_search_method);
	keypoint_detector_->setSearchSurface(surface);
	keypoint_detector_->setInputCloud(pointcloud);
	keypoint_detector_->compute(*pointcloud_keypoints_out);

	pointcloud_keypoints_out->header = pointcloud->header;

	if (retrieve_original_points_metadata_for_detected_keypoints_) {
		std::vector<int> nn_indices(1);
		std::vector<float> nn_dists(1);
		int number_of_neighbors_found = 0;
		
		for (size_t i = 0; i < pointcloud_keypoints_out->size(); ++i) {
			number_of_neighbors_found = surface_search_method->nearestKSearch(pointcloud_keypoints_out->at(i), 1, nn_indices, nn_dists);
			if (number_of_neighbors_found > 0) {
				pointcloud_keypoints_out->at(i) = surface_search_method->getInputCloud()->at(nn_indices[0]);
			}
		}
		
	}

	if (cloud_publisher_ && pointcloud_keypoints_out) { cloud_publisher_->publishPointCloud(*pointcloud_keypoints_out); }
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </KeypointDetector-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <template instantiations>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </template instantiations>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

} /* namespace dynamic_robot_localization */
