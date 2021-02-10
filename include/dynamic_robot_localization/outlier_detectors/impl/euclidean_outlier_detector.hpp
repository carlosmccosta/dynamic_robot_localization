/**\file euclidean_outlier_detector.hpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/outlier_detectors/euclidean_outlier_detector.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
EuclideanOutlierDetector<PointT>::EuclideanOutlierDetector(const std::string& topics_configuration_prefix) : OutlierDetector<PointT>(topics_configuration_prefix), max_inliers_distance_(0.01) {}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <EuclideanOutlierDetector-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
void EuclideanOutlierDetector<PointT>::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace) {
	private_node_handle->param(configuration_namespace + "max_inliers_distance", max_inliers_distance_, 0.01);
	private_node_handle->param(configuration_namespace + "colorize_inliers_based_on_correspondence_distance", colorize_inliers_based_on_correspondence_distance_, false);
	private_node_handle->param(configuration_namespace + "colorize_outliers_with_red_color", colorize_outliers_with_red_color_, false);
	private_node_handle->param(configuration_namespace + "max_curvature_difference", max_curvature_difference_, 0.0);
	private_node_handle->param(configuration_namespace + "max_normals_angular_difference_in_degrees", max_normals_angular_difference_in_degrees_, -30.0);
	private_node_handle->param(configuration_namespace + "max_hsv_color_hue_difference_in_degrees", max_hsv_color_hue_difference_in_degrees_, -30.0);
	private_node_handle->param(configuration_namespace + "max_hsv_color_saturation_difference", max_hsv_color_saturation_difference_, 0.3);
	private_node_handle->param(configuration_namespace + "max_hsv_color_value_difference", max_hsv_color_value_difference_, 0.3);
	OutlierDetector<PointT>::setupConfigurationFromParameterServer(node_handle, private_node_handle, configuration_namespace);
}

template<typename PointT>
bool EuclideanOutlierDetector<PointT>::checkIfCurvatureDifferenceValidationIsEnabled() {
	return (max_curvature_difference_ != 0.0);
}

template<typename PointT>
bool EuclideanOutlierDetector<PointT>::checkIfNormalsDifferenceValidationIsEnabled() {
	return (max_normals_angular_difference_in_degrees_ > 0.0 && max_normals_angular_difference_in_degrees_ <= 180.0);
}

template<typename PointT>
bool EuclideanOutlierDetector<PointT>::checkIfHsvColorDifferenceValidationIsEnabled() {
	return
		(max_hsv_color_hue_difference_in_degrees_ > 0.0 && max_hsv_color_hue_difference_in_degrees_ < 360.0 &&
		max_hsv_color_saturation_difference_ > 0.0 && max_hsv_color_saturation_difference_ <= 1.0 &&
		max_hsv_color_value_difference_ > 0.0 && max_hsv_color_value_difference_ <= 1.0);
}

template<typename PointT>
size_t EuclideanOutlierDetector<PointT>::detectOutliers(typename pcl::search::KdTree<PointT>::Ptr reference_pointcloud_search_method, const pcl::PointCloud<PointT>& ambient_pointcloud,
		typename pcl::PointCloud<PointT>::Ptr& outliers_out, typename pcl::PointCloud<PointT>::Ptr& inliers_out, double& root_mean_square_error_of_inliers_out) {

	bool save_outliers = outliers_out.get() != NULL;
	bool save_inliers = inliers_out.get() != NULL;
	bool curvature_difference_validation_enabled = checkIfCurvatureDifferenceValidationIsEnabled();
	bool normals_difference_validation_enabled = checkIfNormalsDifferenceValidationIsEnabled();
	bool hsv_color_difference_validation_enabled = checkIfHsvColorDifferenceValidationIsEnabled();
	bool difference_validators_enabled = curvature_difference_validation_enabled || normals_difference_validation_enabled || hsv_color_difference_validation_enabled;

	float max_inliers_distance_squared = max_inliers_distance_ * max_inliers_distance_;
	float cos_angle_max_normals_angular_difference_in_radians = normals_difference_validation_enabled ? std::cos(pcl::deg2rad(max_normals_angular_difference_in_degrees_)) : 0.0f;
	root_mean_square_error_of_inliers_out = 0.0;
	size_t number_inliers = 0;

	if (colorize_inliers_based_on_correspondence_distance_) {
		reference_pointcloud_search_method->setSortedResults(true);
	}

	if (max_inliers_distance_ > 0.0) {
		for (size_t i = 0; i < ambient_pointcloud.size(); ++i) {
			PointT point = ambient_pointcloud.points[i];
			std::vector<int> search_indices;
			std::vector<float> search_sqr_distances;
			int number_of_neighbors_found;

			if (difference_validators_enabled) {
				number_of_neighbors_found = reference_pointcloud_search_method->radiusSearch(point, max_inliers_distance_, search_indices, search_sqr_distances);
			} else {
				search_indices.resize(1);
				search_sqr_distances.resize(1);
				number_of_neighbors_found = reference_pointcloud_search_method->nearestKSearch(point, 1, search_indices, search_sqr_distances);
			}

			bool point_is_inlier = false;
			float point_distance_squared = -1.0f;
			if (number_of_neighbors_found > 0 && number_of_neighbors_found == search_indices.size() && number_of_neighbors_found == search_sqr_distances.size()) {
				if (difference_validators_enabled) {
					bool valid_curvature = true;
					bool valid_normal = true;
					bool valid_hsv = true;
					pcl::HSV point_hsv;
					if (hsv_color_difference_validation_enabled) {
						pcl::RGBtoHSV(point.r, point.g, point.b, point_hsv.h, point_hsv.s, point_hsv.v);
					}
					for (int j = 0; j < number_of_neighbors_found; ++j) {
						const PointT& point_neighbor = reference_pointcloud_search_method->getInputCloud()->at(search_indices[j]);

						if (curvature_difference_validation_enabled) {
							valid_curvature = (std::abs(point.curvature - point_neighbor.curvature) <= max_curvature_difference_);
							if (!valid_curvature) continue;
						}

						if (normals_difference_validation_enabled) {
							float cos_angle =
									point.normal_x * point_neighbor.normal_x +
									point.normal_y * point_neighbor.normal_y +
									point.normal_z * point_neighbor.normal_z;
							valid_normal = cos_angle > cos_angle_max_normals_angular_difference_in_radians;
							if (!valid_normal) continue;
						}

						if (hsv_color_difference_validation_enabled) {
							valid_hsv = false;
							pcl::HSV neighbor_hsv;
							pcl::RGBtoHSV(point_neighbor.r, point_neighbor.g, point_neighbor.b, neighbor_hsv.h, neighbor_hsv.s, neighbor_hsv.v);
							float hsv_value_difference = std::abs(neighbor_hsv.v - point_hsv.v);
							if (hsv_value_difference <= max_hsv_color_value_difference_) {
								float hsv_saturation_difference = std::abs(neighbor_hsv.s - point_hsv.s);
								if (hsv_saturation_difference <= max_hsv_color_saturation_difference_) {
									float hue_difference = std::abs(point_hsv.h - neighbor_hsv.h);
									float hue_min_difference = std::min(hue_difference, 360.0f - hue_difference);
									if (hue_min_difference <= max_hsv_color_hue_difference_in_degrees_) {
										valid_hsv = true;
									} else {
										continue;
									}
								} else {
									continue;
								}
							} else {
								continue;
							}
						}

						point_is_inlier = valid_curvature && valid_normal && valid_hsv;
						if (point_is_inlier) {
							point_distance_squared = search_sqr_distances[j];
							break;
						}
					}
				} else {
					if (search_sqr_distances[0] <= max_inliers_distance_squared) {
						point_is_inlier = true;
						point_distance_squared = search_sqr_distances[0];
					}
				}
			}

			if (point_is_inlier) {
				if (point_distance_squared >= 0.0f && colorize_inliers_based_on_correspondence_distance_) {
					float hue = (1.0f - (std::sqrt(point_distance_squared) / max_inliers_distance_)) * 120.0f;
					pcl::HSVtoRGB(hue, 1.0f, 1.0f, point.r, point.g, point.b);
				}
			} else if (colorize_outliers_with_red_color_) {
				point.r = 255;
				point.g = 0;
				point.b = 0;
			}

			if (point_is_inlier) {
				if (save_inliers) { inliers_out->push_back(point); }
				root_mean_square_error_of_inliers_out += point_distance_squared;
				++number_inliers;
			} else {
				if (save_outliers) { outliers_out->push_back(point); }
			}
		}
	}

	if (number_inliers == 0) {
		root_mean_square_error_of_inliers_out = std::numeric_limits<double>::max();
	} else {
		root_mean_square_error_of_inliers_out = std::sqrt(root_mean_square_error_of_inliers_out / (double)number_inliers);
	}

	return ambient_pointcloud.size() - number_inliers;
}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </EuclideanOutlierDetector-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <template instantiations>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </template instantiations>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

} /* namespace dynamic_robot_localization */
