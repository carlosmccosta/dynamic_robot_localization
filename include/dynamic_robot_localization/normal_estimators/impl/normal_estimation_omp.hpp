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
void NormalEstimationOMP<PointT>::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle) {
	int search_k;
	private_node_handle->param("search_k", search_k, 0);
	normal_estimator_.setKSearch(search_k);

	if (search_k <= 0) {
		double search_radius;
		private_node_handle->param("search_radius", search_radius, 0.12);
		normal_estimator_.setRadiusSearch(search_radius);
	}

	NormalEstimator<PointT>::setupConfigurationFromParameterServer(node_handle, private_node_handle);
}

template<typename PointT>
void NormalEstimationOMP<PointT>::estimateNormals(typename pcl::PointCloud<PointT>::Ptr& pointcloud,
		typename pcl::PointCloud<PointT>::Ptr& surface,
		typename pcl::search::KdTree<PointT>::Ptr& surface_search_method,
		tf2::Transform& viewpoint_guess,
		typename pcl::PointCloud<PointT>::Ptr& pointcloud_with_normals_out) {

	/*std::vector<int> indexes;
	pcl::removeNaNFromPointCloud(*pointcloud, *pointcloud, indexes);*/

	normal_estimator_.setSearchMethod(surface_search_method);
	normal_estimator_.setSearchSurface(surface);
	normal_estimator_.setInputCloud(pointcloud);
	normal_estimator_.setViewPoint(viewpoint_guess.getOrigin().getX(), viewpoint_guess.getOrigin().getY(), viewpoint_guess.getOrigin().getZ());
	normal_estimator_.compute(*pointcloud); // adds normals to existing points

	pointcloud_with_normals_out = pointcloud;  // switch pointers

	/*pcl::removeNaNFromPointCloud(*pointcloud_with_normals_out, *pointcloud_with_normals_out, indexes);
	pcl::removeNaNNormalsFromPointCloud(*pointcloud_with_normals_out, *pointcloud_with_normals_out, indexes);*/

	if (NormalEstimator<PointT>::getDisplayNormals()) {
		NormalEstimator<PointT>::displayNormals(pointcloud_with_normals_out);
	}

	ROS_DEBUG_STREAM("NormalEstimationOMP computed " << pointcloud_with_normals_out->points.size() << " normals from a cloud with " << pointcloud->points.size() << " points");
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </NormalEstimationOMP-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// =============================================================================   <private-section>   =========================================================================
// =============================================================================   </private-section>  =========================================================================

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <template instantiations>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </template instantiations>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

} /* namespace dynamic_robot_localization */


