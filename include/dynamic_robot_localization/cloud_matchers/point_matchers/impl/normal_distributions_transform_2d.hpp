/**\file normal_distributions_transform_2d.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/cloud_matchers/point_matchers/normal_distributions_transform_2d.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <NormalDistributionsTransform2D-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
void NormalDistributionsTransform2D<PointT>::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace) {
	typename pcl::Registration<PointT, PointT, float>::Ptr matcher_base(new NormalDistributionsTransform2DDetailed<PointT, PointT>());
	typename pcl::NormalDistributionsTransform2D<PointT, PointT>::Ptr matcher = boost::static_pointer_cast< typename pcl::NormalDistributionsTransform2D<PointT, PointT> >(matcher_base);

	double transformation_rotation_epsilon;
	private_node_handle->param(configuration_namespace + "transformation_rotation_epsilon", transformation_rotation_epsilon, 0.001);
	matcher_base->setTransformationRotationEpsilon(transformation_rotation_epsilon);

	double grid_center_x, grid_center_y;
	private_node_handle->param(configuration_namespace + "grid_center_x", grid_center_x, 0.0);
	private_node_handle->param(configuration_namespace + "grid_center_y", grid_center_y, 0.0);
	matcher->setGridCentre(Eigen::Vector2f(grid_center_x, grid_center_y));

	double grid_step_x, grid_step_y;
	private_node_handle->param(configuration_namespace + "grid_step_x", grid_step_x, 1.0);
	private_node_handle->param(configuration_namespace + "grid_step_y", grid_step_y, 1.0);
	matcher->setGridStep(Eigen::Vector2f(grid_step_x, grid_step_y));

	double grid_extent_x, grid_extent_y;
	private_node_handle->param(configuration_namespace + "grid_extent_x", grid_extent_x, 20.0);
	private_node_handle->param(configuration_namespace + "grid_extent_y", grid_extent_y, 20.0);
	matcher->setGridExtent(Eigen::Vector2f(grid_extent_x, grid_extent_y));

	double grid_optimization_step_size_x, grid_optimization_step_size_y, grid_optimization_step_size_theta;
	private_node_handle->param(configuration_namespace + "grid_optimization_step_size_x", grid_optimization_step_size_x, 1.0);
	private_node_handle->param(configuration_namespace + "grid_optimization_step_size_y", grid_optimization_step_size_y, 1.0);
	private_node_handle->param(configuration_namespace + "grid_optimization_step_size_theta", grid_optimization_step_size_theta, 1.0);
	matcher->setOptimizationStepSize(Eigen::Vector3d(grid_optimization_step_size_x, grid_optimization_step_size_y, grid_optimization_step_size_theta));

	CloudMatcher<PointT>::setCloudMatcher(matcher_base);
	CloudMatcher<PointT>::setupConfigurationFromParameterServer(node_handle, private_node_handle, configuration_namespace);
}


template<typename PointT>
int NormalDistributionsTransform2D<PointT>::getNumberOfRegistrationIterations() {
	if (CloudMatcher<PointT>::cloud_matcher_) {
		typename NormalDistributionsTransform2DDetailed<PointT, PointT>::Ptr matcher = boost::dynamic_pointer_cast< typename dynamic_robot_localization::NormalDistributionsTransform2DDetailed<PointT, PointT> >(CloudMatcher<PointT>::cloud_matcher_);
		if (matcher) { return matcher->getNumberOfRegistrationIterations(); }
	}
	return -1;
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </NormalDistributionsTransform2D-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <template instantiations>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </template instantiations>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

} /* namespace dynamic_robot_localization */
