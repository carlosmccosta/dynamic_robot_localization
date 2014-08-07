/**\file outlier_detector.hpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/outlier_detectors/outlier_detector.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <OutlierDetector-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
void OutlierDetector<PointT>::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace) {
	std::string aligned_pointcloud_outliers_publish_topic, final_param_name;
	if (ros::param::search(private_node_handle->getNamespace() + "/" + configuration_namespace, "aligned_pointcloud_outliers_publish_topic", final_param_name)) {
		private_node_handle->param(final_param_name, aligned_pointcloud_outliers_publish_topic, std::string(""));
	}

	if (!aligned_pointcloud_outliers_publish_topic.empty()) {
		outliers_publisher_ = node_handle->advertise<sensor_msgs::PointCloud2>(aligned_pointcloud_outliers_publish_topic, 5, true);
	}
}


template<typename PointT>
void OutlierDetector<PointT>::publishOutliers(const sensor_msgs::PointCloud2Ptr& outliers) {
	if (outliers->data.size() > 0 && !outliers_publisher_.getTopic().empty()) {
		outliers_publisher_.publish(outliers);
	}
}

template<typename PointT>
sensor_msgs::PointCloud2Ptr OutlierDetector<PointT>::processAndPublishOutliers(typename pcl::search::KdTree<PointT>::Ptr reference_pointcloud_search_method, const pcl::PointCloud<PointT>& ambient_pointcloud) {
	sensor_msgs::PointCloud2Ptr pointcloud_msg;
	if (!outliers_publisher_.getTopic().empty()) {
		pointcloud_msg = processOutliers(reference_pointcloud_search_method, ambient_pointcloud);
		publishOutliers(pointcloud_msg);
	} else {
		pointcloud_msg = sensor_msgs::PointCloud2Ptr(new sensor_msgs::PointCloud2());
	}

	return pointcloud_msg;
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </OutlierDetector-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// =============================================================================   <private-section>   =========================================================================
// =============================================================================   </private-section>  =========================================================================

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <template instantiations>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </template instantiations>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

} /* namespace dynamic_robot_localization */


