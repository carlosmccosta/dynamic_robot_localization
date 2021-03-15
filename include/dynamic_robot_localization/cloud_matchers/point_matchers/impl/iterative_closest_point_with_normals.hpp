/**\file iterative_closest_point_with_normals.hpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/cloud_matchers/point_matchers/iterative_closest_point_with_normals.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <IterativeClosestPointWithNormals-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
void IterativeClosestPointWithNormals<PointT>::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, const std::string& configuration_namespace) {
	typename pcl::Registration<PointT, PointT, float>::Ptr matcher_base(new IterativeClosestPointWithNormalsTimeConstrained<PointT, PointT>());
	typename IterativeClosestPointWithNormalsTimeConstrained<PointT, PointT>::Ptr matcher = std::dynamic_pointer_cast< typename dynamic_robot_localization::IterativeClosestPointWithNormalsTimeConstrained<PointT, PointT> >(matcher_base);

	bool use_symmetric_objective_cost_function;
	bool ensure_normals_with_same_direction_when_using_symmetric_objective_cost_function;
	private_node_handle->param(configuration_namespace + "use_symmetric_objective_cost_function", use_symmetric_objective_cost_function, false);
	private_node_handle->param(configuration_namespace + "ensure_normals_with_same_direction_when_using_symmetric_objective_cost_function", ensure_normals_with_same_direction_when_using_symmetric_objective_cost_function, false);
	matcher->setUseSymmetricObjective(use_symmetric_objective_cost_function);
	matcher->setEnforceSameDirectionNormals(ensure_normals_with_same_direction_when_using_symmetric_objective_cost_function);

	CloudMatcher<PointT>::setCloudMatcher(matcher_base);
	IterativeClosestPoint<PointT>::setupConfigurationFromParameterServer(node_handle, private_node_handle, configuration_namespace);
}

template<typename PointT>
double IterativeClosestPointWithNormals<PointT>::getTransformCloudElapsedTimeMS() {
	typename IterativeClosestPointWithNormalsTimeConstrained<PointT, PointT>::Ptr matcher = std::dynamic_pointer_cast< IterativeClosestPointWithNormalsTimeConstrained<PointT, PointT> >(CloudMatcher<PointT>::cloud_matcher_);
	if (matcher) { return matcher->getTransformCloudElapsedTime(); }
	return -1.0;
}


template<typename PointT>
void IterativeClosestPointWithNormals<PointT>::resetTransformCloudElapsedTime() {
	typename IterativeClosestPointWithNormalsTimeConstrained<PointT, PointT>::Ptr matcher = std::dynamic_pointer_cast< IterativeClosestPointWithNormalsTimeConstrained<PointT, PointT> >(CloudMatcher<PointT>::cloud_matcher_);
	if (matcher) { matcher->resetTransformCloudElapsedTime(); }
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </IterativeClosestPointWithNormals-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <template instantiations>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </template instantiations>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

} /* namespace dynamic_robot_localization */

