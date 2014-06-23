/**\file cloud_matcher.hpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/cloud_matchers/cloud_matcher.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
CloudMatcher<PointT>::CloudMatcher() :
		match_only_keypoints_(false) {}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <CloudMatcher-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

template<typename PointT>
void CloudMatcher<PointT>::setupReferenceCloud(typename pcl::PointCloud<PointT>::Ptr& reference_cloud, typename pcl::search::KdTree<PointT>::Ptr& search_method) {
	cloud_matcher_->setInputTarget(reference_cloud);
	cloud_matcher_->setSearchMethodTarget(search_method);
}


template<typename PointT>
void CloudMatcher<PointT>::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle) {
	private_node_handle->param("match_only_keypoints", match_only_keypoints_, false);

	// subclass must set cloud_matcher_ ptr
	if (cloud_matcher_) {
		double max_correspondence_distance;
		double transformation_epsilon;
		double euclidean_fitness_epsilon;
		int max_number_of_registration_iterations;
		int max_number_of_ransac_iterations;
		double ransac_outlier_rejection_threshold;

		private_node_handle->param("max_correspondence_distance", max_correspondence_distance, 1.0);
		private_node_handle->param("transformation_epsilon", transformation_epsilon, 1e-8);
		private_node_handle->param("euclidean_fitness_epsilon", euclidean_fitness_epsilon, 1e-6);
		private_node_handle->param("max_number_of_registration_iterations", max_number_of_registration_iterations, 500);
		private_node_handle->param("max_number_of_ransac_iterations", max_number_of_ransac_iterations, 500);
		private_node_handle->param("ransac_outlier_rejection_threshold", ransac_outlier_rejection_threshold, 0.05);

		cloud_matcher_->setMaxCorrespondenceDistance(max_correspondence_distance);
		cloud_matcher_->setTransformationEpsilon(transformation_epsilon);
		cloud_matcher_->setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
		cloud_matcher_->setMaximumIterations(max_number_of_registration_iterations);
		cloud_matcher_->setRANSACIterations(max_number_of_ransac_iterations);
		cloud_matcher_->setRANSACOutlierRejectionThreshold(ransac_outlier_rejection_threshold);
	}
}


template<typename PointT>
bool CloudMatcher<PointT>::registerCloud(typename pcl::PointCloud<PointT>::Ptr& ambient_pointcloud,
		typename pcl::search::KdTree<PointT>::Ptr& ambient_pointcloud_search_method,
		typename pcl::PointCloud<PointT>::Ptr& pointcloud_keypoints,
		tf2::Transform& pointcloud_pose_in_out, typename pcl::PointCloud<PointT>::Ptr& pointcloud_registered_out, bool return_aligned_keypoints) {

	processKeypoints(pointcloud_keypoints);

	if (match_only_keypoints_) {
		typename pcl::search::KdTree<PointT>::Ptr pointcloud_keypoints_search_method(new pcl::search::KdTree<PointT>());
		pointcloud_keypoints_search_method->setInputCloud(pointcloud_keypoints);
		cloud_matcher_->setSearchMethodSource(pointcloud_keypoints_search_method);
		cloud_matcher_->setInputSource(pointcloud_keypoints);
	} else {
		cloud_matcher_->setSearchMethodSource(ambient_pointcloud_search_method);
		cloud_matcher_->setInputSource(ambient_pointcloud);
	}

	cloud_matcher_->align(*pointcloud_registered_out);
	if (cloud_matcher_->hasConverged()) {
		tf2::Transform pose_correction;
		laserscan_to_pointcloud::tf_rosmsg_eigen_conversions::transformMatrixToTF2(cloud_matcher_->getFinalTransformation(), pose_correction);
		pointcloud_pose_in_out = pose_correction * pointcloud_pose_in_out;

		if (return_aligned_keypoints && !match_only_keypoints_) {
			pcl::transformPointCloud(*pointcloud_keypoints, *pointcloud_registered_out, cloud_matcher_->getFinalTransformation());
		} else if (!return_aligned_keypoints && match_only_keypoints_) {
			pcl::transformPointCloud(*ambient_pointcloud, *pointcloud_registered_out, cloud_matcher_->getFinalTransformation());
		}

		return true;
	}

	return false;
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </CloudMatcher-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// =============================================================================   <private-section>   =========================================================================
// =============================================================================   </private-section>  =========================================================================

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <template instantiations>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </template instantiations>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

} /* namespace dynamic_robot_localization */


