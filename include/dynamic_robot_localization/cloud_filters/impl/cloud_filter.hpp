/**\file cloud_filter.hpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/cloud_filters/cloud_filter.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


namespace dynamic_robot_localization {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <CloudFilter-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
void CloudFilter<PointT>::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace) {
	cloud_publisher_ = typename CloudPublisher<PointT>::Ptr(new CloudPublisher<PointT>());
	cloud_publisher_->setParameterServerArgumentToLoadTopicName(configuration_namespace + "filtered_cloud_publish_topic");
	cloud_publisher_->setupConfigurationFromParameterServer(node_handle, private_node_handle, configuration_namespace);
}

template<typename PointT>
void CloudFilter<PointT>::filter(const typename pcl::PointCloud<PointT>::Ptr& input_cloud, typename pcl::PointCloud<PointT>::Ptr& output_cloud) {
	size_t number_of_points_in_input_cloud = input_cloud->size();
	filter_->setInputCloud(input_cloud);
	filter_->filter(*output_cloud);

	if (cloud_publisher_ && output_cloud) { cloud_publisher_->publishPointCloud(*output_cloud); }
	ROS_DEBUG_STREAM(filter_name_ << " filter reduced point cloud from " << number_of_points_in_input_cloud << " points to " << output_cloud->size() << " points");
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </CloudFilter-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

} /* namespace dynamic_robot_localization */
