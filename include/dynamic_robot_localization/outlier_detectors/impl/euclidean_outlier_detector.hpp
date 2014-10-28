/**\file euclidean_outlier_detector.cpp
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
EuclideanOutlierDetector<PointT>::EuclideanOutlierDetector() : max_inliers_distance_(0.01) {}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <EuclideanOutlierDetector-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
void EuclideanOutlierDetector<PointT>::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace) {
	private_node_handle->param(configuration_namespace + "max_inliers_distance", max_inliers_distance_, 0.01);
	OutlierDetector<PointT>::setupConfigurationFromParameterServer(node_handle, private_node_handle, configuration_namespace);
}

template<typename PointT>
size_t EuclideanOutlierDetector<PointT>::detectOutliers(typename pcl::search::KdTree<PointT>::Ptr reference_pointcloud_search_method, const pcl::PointCloud<PointT>& ambient_pointcloud,
		sensor_msgs::PointCloud2Ptr& outliers_msg_out, sensor_msgs::PointCloud2Ptr& inliers_msg_out, double& root_mean_square_error_out) {
	std::vector<int> k_indices(1);
	std::vector<float> k_sqr_distances(1);

	root_mean_square_error_out = 0.0;
	size_t number_inliers = 0;
	bool save_outliers = outliers_msg_out.get() != NULL;
	bool save_inliers = inliers_msg_out.get() != NULL;

	PointCloud2Builder outliers_pointcloud_builder;
	outliers_pointcloud_builder.createNewCloud(ambient_pointcloud.header.frame_id, ambient_pointcloud.size());

	PointCloud2Builder inliers_pointcloud_builder;
	inliers_pointcloud_builder.createNewCloud(ambient_pointcloud.header.frame_id, ambient_pointcloud.size());

	for (size_t i = 0; i < ambient_pointcloud.size(); ++i) {
		PointT point = ambient_pointcloud.points[i];
		reference_pointcloud_search_method->nearestKSearch(point, 1, k_indices, k_sqr_distances);
		if (k_sqr_distances[0] > max_inliers_distance_) {
			if (save_outliers) { outliers_pointcloud_builder.addNewPoint(point.x, point.y, point.z); }
		} else {
			if (save_inliers) { inliers_pointcloud_builder.addNewPoint(point.x, point.y, point.z); }
			root_mean_square_error_out += k_sqr_distances[0];
			++number_inliers;
		}
	}

	if (number_inliers == 0) {
		root_mean_square_error_out = std::numeric_limits<double>::max();
	} else {
		root_mean_square_error_out /= number_inliers;
		root_mean_square_error_out = std::sqrt(root_mean_square_error_out);
	}

	if (save_outliers) { outliers_msg_out = outliers_pointcloud_builder.getPointcloudMsg(); }
	if (save_inliers) { inliers_msg_out = inliers_pointcloud_builder.getPointcloudMsg(); }

	return ambient_pointcloud.size() - number_inliers;
}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </EuclideanOutlierDetector-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// =============================================================================   <private-section>   =========================================================================
// =============================================================================   </private-section>  =========================================================================

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <template instantiations>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </template instantiations>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

} /* namespace dynamic_robot_localization */
