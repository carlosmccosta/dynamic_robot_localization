/**\file hsv_segmentation.hpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/cloud_filters/hsv_segmentation.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <HSVSegmentation-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
void HSVSegmentation<PointT>::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace) {
	private_node_handle->param(configuration_namespace + "minimum_hue", minimum_hue_, 0.0);
	private_node_handle->param(configuration_namespace + "maximum_hue", maximum_hue_, 360.0);
	private_node_handle->param(configuration_namespace + "minimum_saturation", minimum_saturation_, 0.0);
	private_node_handle->param(configuration_namespace + "maximum_saturation", maximum_saturation_, 1.0);
	private_node_handle->param(configuration_namespace + "minimum_value", minimum_value_, 0.0);
	private_node_handle->param(configuration_namespace + "maximum_value", maximum_value_, 1.0);
	private_node_handle->param(configuration_namespace + "invert_segmentation", invert_segmentation_, false);
	CloudFilter<PointT>::setupConfigurationFromParameterServer(node_handle, private_node_handle, configuration_namespace);
}

template<typename PointT>
void HSVSegmentation<PointT>::filter(const typename pcl::PointCloud<PointT>::Ptr& input_cloud, typename pcl::PointCloud<PointT>::Ptr& output_cloud) {
	size_t number_of_points_in_input_cloud = input_cloud->size();

	for (size_t i = 0; i < input_cloud->size(); ++i) {
		PointT& point = (*input_cloud)[i];
		float h = 0.0f, s = 0.0f, v = 0.0f;
		pcl::RGBtoHSV(point.r, point.g, point.b, h, s, v);

		bool valid_hue;
		if (minimum_hue_ < maximum_hue_) {
			valid_hue = (h >= minimum_hue_) && (h <= maximum_hue_);
		} else {
			// hue wrap around in the HSV cylinder
			valid_hue = (h <= maximum_hue_) || (h >= minimum_hue_);
		}

		bool valid_saturation = (s >= minimum_saturation_) && (s <= maximum_saturation_);
		bool valid_value = (v >= minimum_value_) && (v <= maximum_value_);
		bool valid_point = valid_hue && valid_saturation && valid_value;

		if ((valid_point && !invert_segmentation_) || (!valid_point && invert_segmentation_)) {
			output_cloud->push_back(point);
		}
	}

	if (CloudFilter<PointT>::cloud_publisher_ && output_cloud) { CloudFilter<PointT>::cloud_publisher_->publishPointCloud(*output_cloud); }
	ROS_DEBUG_STREAM(CloudFilter<PointT>::filter_name_ << " filter reduced point cloud from " << number_of_points_in_input_cloud << " points to " << output_cloud->size() << " points");
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </HSVSegmentation-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

} /* namespace dynamic_robot_localization */
