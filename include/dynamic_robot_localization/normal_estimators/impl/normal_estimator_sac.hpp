/**\file normal_estimator_sac.hpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/normal_estimators/normal_estimator_sac.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <NormalEstimatorSAC-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
void NormalEstimatorSAC<PointT>::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace) {
	int model_type = pcl::SACMODEL_LINE;
	std::string model_type_str;
	private_node_handle->param(configuration_namespace + "model_type", model_type_str, std::string("SACMODEL_LINE"));
	if (model_type_str == "SACMODEL_PLANE") {
		model_type = pcl::SACMODEL_PLANE;
	}
	sac_segmentation_.setModelType(model_type);

	int method_type = pcl::SAC_RANSAC;
	std::string method_type_str;
	private_node_handle->param(configuration_namespace + "method_type", method_type_str, std::string("SAC_RANSAC"));
	if (method_type_str == "SAC_LMEDS") {
		method_type = pcl::SAC_LMEDS;
	} else if (method_type_str == "SAC_MSAC") {
		method_type = pcl::SAC_MSAC;
	} else if (method_type_str == "SAC_RRANSAC") {
		method_type = pcl::SAC_RRANSAC;
	} else if (method_type_str == "SAC_RMSAC") {
		method_type = pcl::SAC_RMSAC;
	} else if (method_type_str == "SAC_MLESAC") {
		method_type = pcl::SAC_MLESAC;
	} else if (method_type_str == "SAC_PROSAC") {
		method_type = pcl::SAC_PROSAC;
	}
	sac_segmentation_.setMethodType(method_type);

	double inlier_distance_threshold;
	private_node_handle->param(configuration_namespace + "inlier_distance_threshold", inlier_distance_threshold, 0.025);
	sac_segmentation_.setDistanceThreshold(inlier_distance_threshold);

	int max_iterations;
	private_node_handle->param(configuration_namespace + "max_iterations", max_iterations, 50);
	sac_segmentation_.setMaxIterations(max_iterations);

	double probability_of_sample_without_outliers;
	private_node_handle->param(configuration_namespace + "probability_of_sample_without_outliers", probability_of_sample_without_outliers, 0.99);
	sac_segmentation_.setProbability(probability_of_sample_without_outliers);

	bool optimize_coefficients;
	private_node_handle->param(configuration_namespace + "optimize_coefficients", optimize_coefficients, true);
	sac_segmentation_.setOptimizeCoefficients(optimize_coefficients);

	double min_model_radius;
	double max_model_radius;
	private_node_handle->param(configuration_namespace + "min_model_radius", min_model_radius, -std::numeric_limits<double>::max());
	private_node_handle->param(configuration_namespace + "max_model_radius", max_model_radius, std::numeric_limits<double>::max());
	if (min_model_radius > 0 && max_model_radius > 0) {
		sac_segmentation_.setRadiusLimits(min_model_radius, max_model_radius);
	}

	private_node_handle->param(configuration_namespace + "random_samples_max_k", random_samples_max_k_, 5);
	private_node_handle->param(configuration_namespace + "random_samples_max_radius", random_samples_max_radius_, 0.05);
	private_node_handle->param(configuration_namespace + "minimum_inliers_percentage", minimum_inliers_percentage_, 0.5);

	NormalEstimator<PointT>::setupConfigurationFromParameterServer(node_handle, private_node_handle, configuration_namespace);
}

template<typename PointT>
void NormalEstimatorSAC<PointT>::estimateNormals(typename pcl::PointCloud<PointT>::Ptr& pointcloud,
		typename pcl::PointCloud<PointT>::Ptr& surface, typename pcl::search::KdTree<PointT>::Ptr& surface_search_method, tf2::Transform& viewpoint_guess,
		typename pcl::PointCloud<PointT>::Ptr& pointcloud_with_normals_out) {
	std::vector<int> indexes;
	pcl::removeNaNFromPointCloud(*pointcloud, *pointcloud, indexes);
	indexes.clear();

	size_t pointcloud_original_size = pointcloud->size();
	if (pointcloud_original_size < 3) { return; }

	pointcloud_with_normals_out = pointcloud;
	sac_segmentation_.setSamplesMaxDist(random_samples_max_radius_, surface_search_method);
	sac_segmentation_.setInputCloud(surface_search_method->getInputCloud());

	float vp_x = viewpoint_guess.getOrigin().x();
	float vp_y = viewpoint_guess.getOrigin().y();
	float vp_z = viewpoint_guess.getOrigin().z();
	PointT& point_1_3 = (*pointcloud_with_normals_out)[(pointcloud_original_size / 3) - 1];
	PointT& point_2_3 = (*pointcloud_with_normals_out)[(2 * pointcloud_original_size / 3) - 1];
	tf2::Vector3 normal_1_3(
			point_1_3.x - vp_x,
			point_1_3.y - vp_y,
			point_1_3.z - vp_z);
	tf2::Vector3 normal_2_3(
			point_2_3.x - vp_x,
			point_2_3.y - vp_y,
			point_2_3.z - vp_z);
	tf2::Vector3 normal_to_viewpoint = normal_1_3.cross(normal_2_3);

	for (size_t i = 0; i < pointcloud_with_normals_out->size(); ++i) {
		PointT& current_point = (*pointcloud_with_normals_out)[i];
		pcl::IndicesPtr nn_indices(new std::vector<int>());
		std::vector<float> nn_distances;
		if (random_samples_max_k_ > 0) {
			surface_search_method->nearestKSearch(current_point, random_samples_max_k_, *nn_indices, nn_distances);
		} else {
			surface_search_method->radiusSearch(current_point, random_samples_max_radius_, *nn_indices, nn_distances);
		}

		bool orient_normal_towards_viewpoint = true;
		if (nn_distances.size() > 2 && nn_indices->size() > 2) {
			sac_segmentation_.setIndices(nn_indices);
			pcl::ModelCoefficients coefficients;
			pcl::PointIndices inliers;
			sac_segmentation_.segment(inliers, coefficients);
			if (!coefficients.values.empty() && inliers.indices.size() > 2 && ((double)nn_indices->size() / (double)inliers.indices.size()) > minimum_inliers_percentage_) {
				if (sac_segmentation_.getModelType() == pcl::SACMODEL_LINE) {
					if (coefficients.values.size() == 6) {
						tf2::Vector3 line_vector(coefficients.values[3], coefficients.values[4], coefficients.values[5]);
						tf2::Vector3 line_normal = line_vector.cross(normal_to_viewpoint);
						line_normal.normalize();
						current_point.normal_x = line_normal.x();
						current_point.normal_y = line_normal.y();
						current_point.normal_z = line_normal.z();
						pcl::flipNormalTowardsViewpoint(current_point, vp_x, vp_y, vp_z, current_point.normal_x, current_point.normal_y, current_point.normal_z);
						orient_normal_towards_viewpoint = false;
					}
				} else if(sac_segmentation_.getModelType() == pcl::SACMODEL_PLANE) {
					if (coefficients.values.size() == 4) {
						current_point.normal_x = coefficients.values[0];
						current_point.normal_y = coefficients.values[1];
						current_point.normal_z = coefficients.values[2];
						pcl::flipNormalTowardsViewpoint(current_point, vp_x, vp_y, vp_z, current_point.normal_x, current_point.normal_y, current_point.normal_z);
						orient_normal_towards_viewpoint = false;
					}
				}
			}
		}

		if (orient_normal_towards_viewpoint) {
			tf2::Vector3 normal(
					vp_x - current_point.x,
					vp_y - current_point.y,
					vp_z - current_point.z);
			normal.normalize();
			current_point.normal_x = normal[0];
			current_point.normal_y = normal[1];
			current_point.normal_z = normal[2];
			current_point.curvature = 0.0;
		}
	}

	pcl::removeNaNFromPointCloud(*pointcloud_with_normals_out, *pointcloud_with_normals_out, indexes);
	indexes.clear();
	pcl::removeNaNNormalsFromPointCloud(*pointcloud_with_normals_out, *pointcloud_with_normals_out, indexes);
	indexes.clear();


	if (NormalEstimator<PointT>::getOccupancyGridMsg()) {
		int search_k = NormalEstimator<PointT>::getOccupancyGridAnalysisK();
		double search_radius = NormalEstimator<PointT>::getOccupancyGridAnalysisRadiusResolutionPercentage();
		if (search_radius > 0) {
			search_radius *= NormalEstimator<PointT>::getOccupancyGridMsg()->info.resolution;
		} else {
			search_radius = NormalEstimator<PointT>::getOccupancyGridAnalysisRadius();
		}

		if (search_k > 0 || search_radius > 0) {
			size_t number_normals_flipped = pointcloud_conversions::flipPointCloudNormalsUsingOccpancyGrid(*(NormalEstimator<PointT>::getOccupancyGridMsg()), *pointcloud_with_normals_out, search_k, search_radius, NormalEstimator<PointT>::getDisplayOccupancyGridPointcloud());
			ROS_DEBUG_STREAM("NormalEstimatorSAC: Flipped " << number_normals_flipped << " normals using OccupancyGrid analysis [ search_k: " << search_k << " | search_radius: " << search_radius << " ]");
		}
	}

	if (pointcloud_with_normals_out->size() > 3 && pointcloud_with_normals_out->size() != pointcloud_original_size) {
		surface_search_method->setInputCloud(pointcloud_with_normals_out);
	}

	ROS_DEBUG_STREAM("NormalEstimatorSAC computed " << pointcloud_with_normals_out->size() << " normals from a cloud with " << pointcloud_original_size << " points");

	NormalEstimator<PointT>::estimateNormals(pointcloud, surface, surface_search_method, viewpoint_guess, pointcloud_with_normals_out);
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </NormalEstimatorSAC-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <template instantiations>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </template instantiations>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

} /* namespace dynamic_robot_localization */


