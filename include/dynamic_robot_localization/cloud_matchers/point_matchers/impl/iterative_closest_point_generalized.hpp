/**\file iterative_closest_point_generalized.hpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/cloud_matchers/point_matchers/iterative_closest_point_generalized.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <IterativeClosestPointGeneralized-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
void IterativeClosestPointGeneralized<PointT>::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace) {
	typename pcl::Registration<PointT, PointT>::Ptr matcher_base(new dynamic_robot_localization::IterativeClosestPointGeneralizedTimeConstrained<PointT, PointT>());
	typename dynamic_robot_localization::IterativeClosestPointGeneralizedTimeConstrained<PointT, PointT>::Ptr matcher = boost::dynamic_pointer_cast< typename dynamic_robot_localization::IterativeClosestPointGeneralizedTimeConstrained<PointT, PointT> >(matcher_base);

	double rotation_epsilon;
	private_node_handle->param(configuration_namespace + "rotation_epsilon", rotation_epsilon, 0.002);
	matcher->setRotationEpsilon(rotation_epsilon);

	int correspondence_randomness;
	private_node_handle->param(configuration_namespace + "correspondence_randomness", correspondence_randomness, 20);
	matcher->setCorrespondenceRandomness(correspondence_randomness);

	int maximum_optimizer_iterations;
	private_node_handle->param(configuration_namespace + "maximum_optimizer_iterations", maximum_optimizer_iterations, 20);
	matcher->setMaximumOptimizerIterations(maximum_optimizer_iterations);

	CloudMatcher<PointT>::setCloudMatcher(matcher_base);
	IterativeClosestPoint<PointT>::setupConfigurationFromParameterServer(node_handle, private_node_handle, configuration_namespace);
}

template<typename PointT>
double IterativeClosestPointGeneralized<PointT>::getTransformCloudElapsedTimeMS() {
	typename IterativeClosestPointGeneralizedTimeConstrained<PointT, PointT>::Ptr matcher = boost::dynamic_pointer_cast< IterativeClosestPointGeneralizedTimeConstrained<PointT, PointT> >(CloudMatcher<PointT>::cloud_matcher_);
	if (matcher) { return matcher->getTransformCloudElapsedTime(); }
	return -1.0;
}


template<typename PointT>
void IterativeClosestPointGeneralized<PointT>::resetTransformCloudElapsedTime() {
	typename IterativeClosestPointGeneralizedTimeConstrained<PointT, PointT>::Ptr matcher = boost::dynamic_pointer_cast< IterativeClosestPointGeneralizedTimeConstrained<PointT, PointT> >(CloudMatcher<PointT>::cloud_matcher_);
	if (matcher) { matcher->resetTransformCloudElapsedTime(); }
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </IterativeClosestPointWithNormals-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <template instantiations>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </template instantiations>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

} /* namespace dynamic_robot_localization */

