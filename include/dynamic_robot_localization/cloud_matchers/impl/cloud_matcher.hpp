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
		match_only_keypoints_(false),
		display_cloud_aligment_(false),
		maximum_number_of_displayed_correspondences_(0) {}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <CloudMatcher-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
void CloudMatcher<PointT>::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace) {
	std::string final_param_name;
	std::string search_namespace = private_node_handle->getNamespace() + "/" + configuration_namespace;
	if (ros::param::search(search_namespace, "match_only_keypoints", final_param_name)) { private_node_handle->param(final_param_name, match_only_keypoints_, false); }
	if (ros::param::search(search_namespace, "display_cloud_aligment", final_param_name)) { private_node_handle->param(final_param_name, display_cloud_aligment_, false); }
	if (ros::param::search(search_namespace, "maximum_number_of_displayed_correspondences", final_param_name)) { private_node_handle->param(final_param_name, maximum_number_of_displayed_correspondences_, 0); } // show all

	// subclass must set cloud_matcher_ ptr
	if (cloud_matcher_) {
		double max_correspondence_distance;
		double transformation_epsilon;
		double euclidean_fitness_epsilon;
		int max_number_of_registration_iterations;
		int max_number_of_ransac_iterations;
		double ransac_outlier_rejection_threshold;

		if (ros::param::search(search_namespace, "max_correspondence_distance", final_param_name)) { private_node_handle->param(final_param_name, max_correspondence_distance, 0.1); }
		if (ros::param::search(search_namespace, "transformation_epsilon", final_param_name)) { private_node_handle->param(final_param_name, transformation_epsilon, 1e-8); }
		if (ros::param::search(search_namespace, "euclidean_fitness_epsilon", final_param_name)) { private_node_handle->param(final_param_name, euclidean_fitness_epsilon, 1e-6); }
		if (ros::param::search(search_namespace, "max_number_of_registration_iterations", final_param_name)) { private_node_handle->param(final_param_name, max_number_of_registration_iterations, 250); }
		if (ros::param::search(search_namespace, "max_number_of_ransac_iterations", final_param_name)) { private_node_handle->param(final_param_name, max_number_of_ransac_iterations, 250); }
		if (ros::param::search(search_namespace, "ransac_outlier_rejection_threshold", final_param_name)) { private_node_handle->param(final_param_name, ransac_outlier_rejection_threshold, 0.05); }

		cloud_matcher_->setMaxCorrespondenceDistance(max_correspondence_distance);
		cloud_matcher_->setTransformationEpsilon(transformation_epsilon);
		cloud_matcher_->setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
		cloud_matcher_->setMaximumIterations(max_number_of_registration_iterations);
		cloud_matcher_->setRANSACIterations(max_number_of_ransac_iterations);
		cloud_matcher_->setRANSACOutlierRejectionThreshold(ransac_outlier_rejection_threshold);

		setupRegistrationVisualizer();
	}
}


template<typename PointT>
void CloudMatcher<PointT>::setupReferenceCloud(typename pcl::PointCloud<PointT>::Ptr& reference_cloud, typename pcl::PointCloud<PointT>::Ptr& reference_cloud_keypoints,
		typename pcl::search::KdTree<PointT>::Ptr& search_method) {
	// subclass must set cloud_matcher_ ptr
	if (cloud_matcher_) {
		cloud_matcher_->setInputTarget(reference_cloud);
		cloud_matcher_->setSearchMethodTarget(search_method);
	}

	if (registration_visualizer_) {
		registration_visualizer_->setTargetCloud(*reference_cloud);
	}
}


template<typename PointT>
bool CloudMatcher<PointT>::registerCloud(typename pcl::PointCloud<PointT>::Ptr& ambient_pointcloud,
		typename pcl::search::KdTree<PointT>::Ptr& ambient_pointcloud_search_method,
		typename pcl::PointCloud<PointT>::Ptr& pointcloud_keypoints,
		tf2::Transform& pose_correction_out, typename pcl::PointCloud<PointT>::Ptr& pointcloud_registered_out, bool return_aligned_keypoints) {

	// subclass must set cloud_matcher_ ptr
	if (!cloud_matcher_) {
		return false;
	}

	initializeKeypointProcessing();

	if (match_only_keypoints_) {
		ROS_DEBUG_STREAM("Registering cloud with " << pointcloud_keypoints->size() << " points");
		typename pcl::search::KdTree<PointT>::Ptr pointcloud_keypoints_search_method(new pcl::search::KdTree<PointT>());
		pointcloud_keypoints_search_method->setInputCloud(pointcloud_keypoints);
		cloud_matcher_->setSearchMethodSource(pointcloud_keypoints_search_method);
		cloud_matcher_->setInputSource(pointcloud_keypoints);
		if (registration_visualizer_) { registration_visualizer_->setSourceCloud(*pointcloud_keypoints); }
	} else {
		ROS_DEBUG_STREAM("Registering cloud with " << ambient_pointcloud->size() << " points");
		cloud_matcher_->setSearchMethodSource(ambient_pointcloud_search_method);
		cloud_matcher_->setInputSource(ambient_pointcloud);
		if (registration_visualizer_) { registration_visualizer_->setSourceCloud(*ambient_pointcloud); }
	}

	processKeypoints(pointcloud_keypoints, ambient_pointcloud, ambient_pointcloud_search_method);

	cloud_matcher_->align(*pointcloud_registered_out);

	if (cloud_matcher_->hasConverged()) {
		laserscan_to_pointcloud::tf_rosmsg_eigen_conversions::transformMatrixToTF2(cloud_matcher_->getFinalTransformation(), pose_correction_out);

		if (return_aligned_keypoints && !match_only_keypoints_) {
			pcl::transformPointCloud(*pointcloud_keypoints, *pointcloud_registered_out, cloud_matcher_->getFinalTransformation());
		} else if (!return_aligned_keypoints && match_only_keypoints_) {
			pcl::transformPointCloud(*ambient_pointcloud, *pointcloud_registered_out, cloud_matcher_->getFinalTransformation());
		}

		// if publisher available, send aligned cloud
		if (cloud_publisher_) {
			cloud_publisher_->publishPointCloud(*pointcloud_registered_out);
		}

		return true;
	}

	return false;
}


template<typename PointT>
void CloudMatcher<PointT>::setDisplayCloudAligment(bool display_cloud_aligment) {
	display_cloud_aligment_ = display_cloud_aligment;
}


template<typename PointT>
void CloudMatcher<PointT>::setupRegistrationVisualizer() {
	if (cloud_matcher_ && !registration_visualizer_ && display_cloud_aligment_) {
		registration_visualizer_ = boost::shared_ptr< RegistrationVisualizer<PointT, PointT> >(new RegistrationVisualizer<PointT, PointT>());
		registration_visualizer_->setMaximumDisplayedCorrespondences(maximum_number_of_displayed_correspondences_);
		registration_visualizer_->setRegistration(*cloud_matcher_);

		registration_visualizer_->startDisplay();
		ROS_DEBUG_STREAM("RegistrationVisualizer activated with " << maximum_number_of_displayed_correspondences_ << " number_maximum_displayed_correspondences");
	}
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


