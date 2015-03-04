/**\file normal_estimation_omp.hpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/normal_estimators/normal_estimation_omp.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <NormalEstimationOMP-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
void NormalEstimationOMP<PointT>::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace) {
	int search_k;
	private_node_handle->param(configuration_namespace + "search_k", search_k, 0);
	normal_estimator_.setKSearch(search_k);

	if (search_k <= 0) {
		double search_radius;
		private_node_handle->param(configuration_namespace + "search_radius", search_radius, 0.12);
		normal_estimator_.setRadiusSearch(search_radius);
	}

	NormalEstimator<PointT>::setupConfigurationFromParameterServer(node_handle, private_node_handle, configuration_namespace);
}

template<typename PointT>
void NormalEstimationOMP<PointT>::estimateNormals(typename pcl::PointCloud<PointT>::Ptr& pointcloud,
		typename pcl::PointCloud<PointT>::Ptr& surface,
		typename pcl::search::KdTree<PointT>::Ptr& surface_search_method,
		tf2::Transform& viewpoint_guess,
		typename pcl::PointCloud<PointT>::Ptr& pointcloud_with_normals_out) {
	size_t pointcloud_original_size = pointcloud->size();
	if (pointcloud_original_size < 3) { return; }

	std::vector<int> indexes;
	pcl::removeNaNFromPointCloud(*pointcloud, *pointcloud, indexes);
	indexes.clear();

	normal_estimator_.setSearchMethod(surface_search_method);
	if (surface) { normal_estimator_.setSearchSurface(surface); }
	normal_estimator_.setInputCloud(pointcloud);
	normal_estimator_.setViewPoint(viewpoint_guess.getOrigin().getX(), viewpoint_guess.getOrigin().getY(), viewpoint_guess.getOrigin().getZ());
	normal_estimator_.compute(*pointcloud); // adds normals to existing points

	pointcloud_with_normals_out = pointcloud;  // switch pointers

	pcl::removeNaNFromPointCloud(*pointcloud_with_normals_out, *pointcloud_with_normals_out, indexes);
	indexes.clear();
	pcl::removeNaNNormalsFromPointCloud(*pointcloud_with_normals_out, *pointcloud_with_normals_out, indexes);
	indexes.clear();

	ROS_DEBUG_STREAM("NormalEstimationOMP computed " << pointcloud_with_normals_out->size() << " normals from a cloud with " << pointcloud_original_size << " points");

	NormalEstimator<PointT>::estimateNormals(pointcloud, surface, surface_search_method, viewpoint_guess, pointcloud_with_normals_out);

	if (pointcloud_with_normals_out->size() > 3 && pointcloud_with_normals_out->size() != pointcloud_original_size) {
		surface_search_method->setInputCloud(pointcloud_with_normals_out);
	}
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </NormalEstimationOMP-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================


} /* namespace dynamic_robot_localization */


