/**\file principal_curvatures_estimation.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/curvature_estimators/principal_curvatures_estimation.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <PrincipalCurvaturesEstimation-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
void PrincipalCurvaturesEstimation<PointT>::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace) {
	int search_k;
	private_node_handle->param(configuration_namespace + "search_k", search_k, 0);
	curvature_estimator_.setKSearch(search_k);

	if (search_k <= 0) {
		double search_radius;
		private_node_handle->param(configuration_namespace + "search_radius", search_radius, 0.12);
		curvature_estimator_.setRadiusSearch(search_radius);
	}

	private_node_handle->param(configuration_namespace + "update_normals_with_principal_component_directions", update_normals_with_principal_component_directions_, false);

	std::string upsample_method_str;
	curvature_type_ = CURVATURE_TYPE_MEAN;
	private_node_handle->param(configuration_namespace + "curvature_type", upsample_method_str, std::string("CURVATURE_TYPE_MEAN"));
	if (upsample_method_str == "CURVATURE_TYPE_GAUSSIAN") {
		curvature_type_ = CURVATURE_TYPE_GAUSSIAN;
	}

	CurvatureEstimator<PointT>::setupConfigurationFromParameterServer(node_handle, private_node_handle, configuration_namespace);
}

template<typename PointT>
void PrincipalCurvaturesEstimation<PointT>::estimatePointsCurvature(typename pcl::PointCloud<PointT>::Ptr& pointcloud, typename pcl::search::KdTree<PointT>::Ptr& search_method) {
	curvature_estimator_.setSearchMethod(search_method);
	curvature_estimator_.setInputCloud(pointcloud);
	curvature_estimator_.setInputNormals(pointcloud);

	pcl::PointCloud<pcl::PrincipalCurvatures> principal_curvatures;
	curvature_estimator_.compute(principal_curvatures);

	for (size_t i = 0; (i < pointcloud->size()) && (i < principal_curvatures.size()); ++i) {
		if (curvature_type_ == CURVATURE_TYPE_GAUSSIAN) {
			(*pointcloud)[i].curvature = principal_curvatures[i].pc1 * principal_curvatures[i].pc2;
		} else {
			(*pointcloud)[i].curvature = (principal_curvatures[i].pc1 + principal_curvatures[i].pc2) * 0.5;
		}

		if (update_normals_with_principal_component_directions_) {
			(*pointcloud)[i].normal_x = principal_curvatures[i].principal_curvature_x;
			(*pointcloud)[i].normal_y = principal_curvatures[i].principal_curvature_y;
			(*pointcloud)[i].normal_z = principal_curvatures[i].principal_curvature_z;
		}
	}

	ROS_DEBUG_STREAM("PrincipalCurvaturesEstimation computed curvature information for a point cloud with " << pointcloud->size() << " points");

	CurvatureEstimator<PointT>::estimatePointsCurvature(pointcloud, search_method);
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </PrincipalCurvaturesEstimation-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

} /* namespace dynamic_robot_localization */

