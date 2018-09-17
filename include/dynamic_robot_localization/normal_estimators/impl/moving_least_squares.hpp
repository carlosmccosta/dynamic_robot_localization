/**\file moving_least_squares.hpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/normal_estimators/moving_least_squares.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <MovingLeastSquares-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
void MovingLeastSquares<PointT>::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace) {
	bool compute_normals;
	private_node_handle->param(configuration_namespace + "compute_normals", compute_normals, true);
	normal_estimator_.setComputeNormals(compute_normals);

	int polynomial_order;
	private_node_handle->param(configuration_namespace + "polynomial_order", polynomial_order, 2);
	normal_estimator_.setPolynomialOrder(polynomial_order);

	double search_radius;
	private_node_handle->param(configuration_namespace + "search_radius", search_radius, 0.12);
	normal_estimator_.setSearchRadius(search_radius);

	double sqr_gauss_param;
	private_node_handle->param(configuration_namespace + "sqr_gauss_param", sqr_gauss_param, search_radius * search_radius);
	normal_estimator_.setSqrGaussParam(sqr_gauss_param);

	typename pcl::MovingLeastSquares<PointT, PointT>::UpsamplingMethod upsample_method = pcl::MovingLeastSquares<PointT, PointT>::NONE;
	std::string upsample_method_str;
	private_node_handle->param(configuration_namespace + "upsample_method", upsample_method_str, std::string("NONE"));
	if (upsample_method_str == "SAMPLE_LOCAL_PLANE") {
		upsample_method = pcl::MovingLeastSquares<PointT, PointT>::SAMPLE_LOCAL_PLANE;
	} else if (upsample_method_str == "RANDOM_UNIFORM_DENSITY") {
		upsample_method = pcl::MovingLeastSquares<PointT, PointT>::RANDOM_UNIFORM_DENSITY;
	} else if (upsample_method_str == "VOXEL_GRID_DILATION") {
		upsample_method = pcl::MovingLeastSquares<PointT, PointT>::VOXEL_GRID_DILATION;
	}
	normal_estimator_.setUpsamplingMethod(upsample_method);

	double upsampling_radius;
	private_node_handle->param(configuration_namespace + "upsampling_radius", upsampling_radius, 0.05);
	normal_estimator_.setUpsamplingRadius(upsampling_radius);

	double upsampling_step;
	private_node_handle->param(configuration_namespace + "upsampling_step", upsampling_step, 0.01);
	normal_estimator_.setUpsamplingStepSize(upsampling_step);

	int desired_num_points_in_radius;
	private_node_handle->param(configuration_namespace + "desired_num_points_in_radius", desired_num_points_in_radius, 5);
	normal_estimator_.setPointDensity(desired_num_points_in_radius);

	double dilation_voxel_size;
	private_node_handle->param(configuration_namespace + "dilation_voxel_size", dilation_voxel_size, 0.01);
	normal_estimator_.setDilationVoxelSize(dilation_voxel_size);

	int dilation_iterations;
	private_node_handle->param(configuration_namespace + "dilation_iterations", dilation_iterations, 1);
	normal_estimator_.setDilationIterations(dilation_iterations);

	NormalEstimator<PointT>::setupConfigurationFromParameterServer(node_handle, private_node_handle, configuration_namespace);
}

template<typename PointT>
void MovingLeastSquares<PointT>::estimateNormals(typename pcl::PointCloud<PointT>::Ptr& pointcloud,
		typename pcl::PointCloud<PointT>::Ptr& surface,
		typename pcl::search::KdTree<PointT>::Ptr& surface_search_method,
		tf2::Transform& viewpoint_guess,
		typename pcl::PointCloud<PointT>::Ptr& pointcloud_with_normals_out) {
	size_t pointcloud_original_size = pointcloud->size();
	if (pointcloud_original_size < 3) { return; }

	std::vector<int> indexes;
	pcl::removeNaNFromPointCloud(*pointcloud, *pointcloud, indexes);
	indexes.clear();

	typename pcl::PointCloud<PointT>::Ptr pointcloud_resampled(new typename pcl::PointCloud<PointT>());

	normal_estimator_.setSearchMethod(surface_search_method);
	normal_estimator_.setInputCloud(pointcloud);
	normal_estimator_.process(*pointcloud_resampled);

	pcl::removeNaNFromPointCloud(*pointcloud_resampled, *pointcloud_with_normals_out, indexes);
	indexes.clear();
	pcl::removeNaNNormalsFromPointCloud(*pointcloud_with_normals_out, *pointcloud_with_normals_out, indexes);
	indexes.clear();

	ROS_DEBUG_STREAM("MovingLeastSquares computed " << pointcloud_with_normals_out->size() << " normals from a cloud with " << pointcloud_original_size << " points");

	NormalEstimator<PointT>::estimateNormals(pointcloud, surface, surface_search_method, viewpoint_guess, pointcloud_with_normals_out);

	if (pointcloud_with_normals_out->size() > 3 && pointcloud_with_normals_out->size() != pointcloud_original_size) {
		surface_search_method->setInputCloud(pointcloud_with_normals_out);
	}
}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </MovingLeastSquares-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <template instantiations>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </template instantiations>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

} /* namespace dynamic_robot_localization */

