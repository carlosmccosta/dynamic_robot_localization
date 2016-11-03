/**\file iterative_closest_point.hpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/cloud_matchers/point_matchers/iterative_closest_point.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <IterativeClosestPoint-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
void IterativeClosestPoint<PointT>::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace) {
	private_node_handle->param(configuration_namespace + "convergence_absolute_mse_threshold", convergence_absolute_mse_threshold_, 1e-12);
	private_node_handle->param(configuration_namespace + "convergence_rotation_threshold", convergence_rotation_threshold_, 0.0);
	private_node_handle->param(configuration_namespace + "convergence_max_iterations_similar_transforms", convergence_max_iterations_similar_transforms_, 0);
	double convergence_time_limit_seconds;
	private_node_handle->param(configuration_namespace + "convergence_time_limit_seconds", convergence_time_limit_seconds, -1.0);
	private_node_handle->param(configuration_namespace + "convergence_time_limit_seconds_as_mean_convergence_time_percentage", convergence_time_limit_seconds_as_mean_convergence_time_percentage_, 3.0);
	private_node_handle->param(configuration_namespace + "minimum_number_of_convergence_time_measurements_to_adjust_convergence_time_limit", minimum_number_of_convergence_time_measurements_to_adjust_convergence_time_limit_, 25);

	convergence_time_limit_seconds_ = convergence_time_limit_seconds;

	if (convergence_time_limit_seconds <= 0.0) {
		convergence_time_limit_seconds = std::numeric_limits<double>::max();
	}

	if (!CloudMatcher<PointT>::cloud_matcher_) {
//		CloudMatcher<PointT>::cloud_matcher_ = typename pcl::Registration<PointT, PointT, float>::Ptr(new pcl::IterativeClosestPoint<PointT, PointT, float>());
		CloudMatcher<PointT>::cloud_matcher_ = typename pcl::Registration<PointT, PointT, float>::Ptr(new IterativeClosestPointTimeConstrained<PointT, PointT, float>(convergence_time_limit_seconds));
	}
	typename DefaultConvergenceCriteriaWithTime<float>::Ptr convergence_criteria = getConvergenceCriteria();
	if (convergence_criteria) {
		convergence_criteria->setConvergenceTimeLimitSeconds(convergence_time_limit_seconds);
		convergence_criteria->setAbsoluteMSE(convergence_absolute_mse_threshold_);
		convergence_criteria->setConvergenceRotationThreshold(convergence_rotation_threshold_);
		convergence_criteria->setMaximumIterationsSimilarTransforms(convergence_max_iterations_similar_transforms_);
	}

	ROS_DEBUG_STREAM("Setting a registration time limit of " << convergence_time_limit_seconds << " seconds to " << CloudMatcher<PointT>::cloud_matcher_->getClassName() << " algorithm");

	bool use_reciprocal_correspondences;
	private_node_handle->param(configuration_namespace + "use_reciprocal_correspondences", use_reciprocal_correspondences, false);
	typename pcl::IterativeClosestPoint<PointT, PointT, float>::Ptr matcher = boost::dynamic_pointer_cast< typename pcl::IterativeClosestPoint<PointT, PointT, float> >(CloudMatcher<PointT>::cloud_matcher_);
	matcher->setUseReciprocalCorrespondences(use_reciprocal_correspondences);
	CloudMatcher<PointT>::setForceNoRecomputeReciprocal(!use_reciprocal_correspondences);

	if (convergence_rotation_threshold_ > 0)
		matcher->setTransformationRotationEpsilon(convergence_rotation_threshold_);

	CloudMatcher<PointT>::setupConfigurationFromParameterServer(node_handle, private_node_handle, configuration_namespace);
}


template<typename PointT>
bool IterativeClosestPoint<PointT>::registerCloud(typename pcl::PointCloud<PointT>::Ptr& ambient_pointcloud, typename pcl::search::KdTree<PointT>::Ptr& ambient_pointcloud_search_method, typename pcl::PointCloud<PointT>::Ptr& pointcloud_keypoints,
		tf2::Transform& best_pose_correction_out, std::vector<tf2::Transform>& accepted_pose_corrections_out, typename pcl::PointCloud<PointT>::Ptr& pointcloud_registered_out, bool return_aligned_keypoints) {
	typename DefaultConvergenceCriteriaWithTime<float>::Ptr convergence_criteria = getConvergenceCriteria();
	if (convergence_criteria) {
		convergence_criteria->resetConvergenceTimer();
	}

	if (CloudMatcher<PointT>::registerCloud(ambient_pointcloud, ambient_pointcloud_search_method, pointcloud_keypoints, best_pose_correction_out, accepted_pose_corrections_out, pointcloud_registered_out, return_aligned_keypoints)) {
		cumulative_sum_of_convergence_time_ += convergence_criteria->getConvergenceElaspedTime();
		++number_of_convergence_time_measurements;

		if (convergence_time_limit_seconds_ > 0.0 &&
				convergence_time_limit_seconds_as_mean_convergence_time_percentage_ > 0.0 &&
				minimum_number_of_convergence_time_measurements_to_adjust_convergence_time_limit_ > 0 &&
				number_of_convergence_time_measurements > minimum_number_of_convergence_time_measurements_to_adjust_convergence_time_limit_) {
			double updated_convergence_time_limit_seconds = std::min(convergence_time_limit_seconds_, (cumulative_sum_of_convergence_time_ / number_of_convergence_time_measurements) * convergence_time_limit_seconds_as_mean_convergence_time_percentage_);
			if (updated_convergence_time_limit_seconds > 0.0 && convergence_criteria) {
				convergence_criteria->setConvergenceTimeLimitSeconds(updated_convergence_time_limit_seconds);
				ROS_DEBUG_STREAM("Updating " << CloudMatcher<PointT>::cloud_matcher_->getClassName()  << " convergence time limit to " << updated_convergence_time_limit_seconds);
			}
		}
		return true;
	} else {
		return false;
	}
}


template<typename PointT>
int IterativeClosestPoint<PointT>::getNumberOfRegistrationIterations() {
	typename DefaultConvergenceCriteriaWithTime<float>::Ptr convergence_criteria = getConvergenceCriteria();
	if (convergence_criteria) {
		return convergence_criteria->getNumberOfRegistrationIterations();
	} else {
		return -1;
	}
}


template<typename PointT>
double IterativeClosestPoint<PointT>::getRootMeanSquareErrorOfRegistrationCorrespondences() {
	typename DefaultConvergenceCriteriaWithTime<float>::Ptr convergence_criteria = getConvergenceCriteria();
	if (convergence_criteria) {
		return convergence_criteria->getRootMeanSquareErrorOfRegistrationCorrespondences();
	} else {
		return -1.0;
	}
}


template<typename PointT>
int IterativeClosestPoint<PointT>::getNumberCorrespondencesInLastRegistrationIteration() {
	typename DefaultConvergenceCriteriaWithTime<float>::Ptr convergence_criteria = getConvergenceCriteria();
	if (convergence_criteria) {
		return convergence_criteria->getNumberCorrespondences();
	} else {
		return -1.0;
	}
}


template<typename PointT>
typename DefaultConvergenceCriteriaWithTime<float>::Ptr IterativeClosestPoint<PointT>::getConvergenceCriteria() {
	typename DefaultConvergenceCriteriaWithTime<float>::Ptr convergence_criteria;
	if (CloudMatcher<PointT>::cloud_matcher_) {
		typename pcl::IterativeClosestPoint<PointT, PointT, float>::Ptr matcher = boost::dynamic_pointer_cast< typename pcl::IterativeClosestPoint<PointT, PointT, float> >(CloudMatcher<PointT>::cloud_matcher_);
		if (matcher) { convergence_criteria = boost::dynamic_pointer_cast< typename dynamic_robot_localization::DefaultConvergenceCriteriaWithTime<float> >(matcher->getConvergeCriteria()); }
	}

	return convergence_criteria;
}

template<typename PointT>
std::string IterativeClosestPoint<PointT>::getMatcherConvergenceState() {
	typename DefaultConvergenceCriteriaWithTime<float>::Ptr convergence_criteria = getConvergenceCriteria();
	if (convergence_criteria) {
		return convergence_criteria->getConvergenceStateString();
	} else {
		return "";
	}
}

template<typename PointT>
double IterativeClosestPoint<PointT>::getTransformCloudElapsedTimeMS() {
	typename IterativeClosestPointTimeConstrained<PointT, PointT>::Ptr matcher = boost::dynamic_pointer_cast< IterativeClosestPointTimeConstrained<PointT, PointT> >(CloudMatcher<PointT>::cloud_matcher_);
	if (matcher) { return matcher->getTransformCloudElapsedTime(); }
	return -1.0;
}


template<typename PointT>
void IterativeClosestPoint<PointT>::resetTransformCloudElapsedTime() {
	typename IterativeClosestPointTimeConstrained<PointT, PointT>::Ptr matcher = boost::dynamic_pointer_cast< IterativeClosestPointTimeConstrained<PointT, PointT> >(CloudMatcher<PointT>::cloud_matcher_);
	if (matcher) { matcher->resetTransformCloudElapsedTime(); }
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </IterativeClosestPoint-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <template instantiations>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </template instantiations>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

} /* namespace dynamic_robot_localization */


