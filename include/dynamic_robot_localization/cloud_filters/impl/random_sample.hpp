/**\file random_sample.hpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/cloud_filters/random_sample.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <RandomSample-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
void RandomSample<PointT>::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace) {
	typename pcl::Filter<PointT>::Ptr filter_base(new pcl::RandomSample<PointT>());
	typename pcl::RandomSample<PointT>::Ptr filter = boost::static_pointer_cast< typename pcl::RandomSample<PointT> >(filter_base);

	int number_of_random_samples;
	private_node_handle->param(configuration_namespace + "number_of_random_samples", number_of_random_samples, 250);
	filter->setSample(number_of_random_samples);

	bool invert_sampling;
	private_node_handle->param(configuration_namespace + "invert_sampling", invert_sampling, false);
	filter->setNegative(invert_sampling);

	CloudFilter<PointT>::setFilter(filter_base);
	CloudFilter<PointT>::setupConfigurationFromParameterServer(node_handle, private_node_handle, configuration_namespace);
}

template<typename PointT>
void RandomSample<PointT>::filter(const typename pcl::PointCloud<PointT>::Ptr& input_cloud, typename pcl::PointCloud<PointT>::Ptr& output_cloud) {
	size_t number_of_points_in_input_cloud = input_cloud->size();

	typename pcl::RandomSample<PointT>::Ptr filter = boost::static_pointer_cast< typename pcl::RandomSample<PointT> >(CloudFilter<PointT>::filter_);
	if (filter->getSample() >= number_of_points_in_input_cloud) {
		if (filter->getNegative()) {
			output_cloud = typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
			output_cloud->header = input_cloud->header;
		} else {
			output_cloud = input_cloud;
		}
	} else {
		CloudFilter<PointT>::filter_->setInputCloud(input_cloud);
		CloudFilter<PointT>::filter_->filter(*output_cloud);
	}


	if (CloudFilter<PointT>::getCloudPublisher() && output_cloud) { CloudFilter<PointT>::getCloudPublisher()->publishPointCloud(*output_cloud); }
	ROS_DEBUG_STREAM(CloudFilter<PointT>::filter_name_ << " filter reduced point cloud from " << number_of_points_in_input_cloud << " points to " << output_cloud->size() << " points");
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </RandomSample-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

} /* namespace dynamic_robot_localization */
