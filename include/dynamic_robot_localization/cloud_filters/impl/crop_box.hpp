/**\file crop_box.hpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/cloud_filters/crop_box.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <RadiusOutlierRemoval-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
void CropBox<PointT>::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace) {
	typename pcl::Filter<PointT>::Ptr filter_base(new pcl::CropBox<PointT>());
	typename pcl::CropBox<PointT>::Ptr filter = boost::static_pointer_cast< typename pcl::CropBox<PointT> >(filter_base);

	double box_min_x, box_min_y, box_min_z;
	private_node_handle->param(configuration_namespace + "box_min_x", box_min_x, -10.0);
	private_node_handle->param(configuration_namespace + "box_min_y", box_min_y, -10.0);
	private_node_handle->param(configuration_namespace + "box_min_z", box_min_z, -10.0);
	filter->setMin(Eigen::Vector4f(box_min_x, box_min_y, box_min_z, 1));

	double box_max_x, box_max_y, box_max_z;
	private_node_handle->param(configuration_namespace + "box_max_x", box_max_x, 10.0);
	private_node_handle->param(configuration_namespace + "box_max_y", box_max_y, 10.0);
	private_node_handle->param(configuration_namespace + "box_max_z", box_max_z, 10.0);
	filter->setMax(Eigen::Vector4f(box_max_x, box_max_y, box_max_z, 1));

	double box_translation_x, box_translation_y, box_translation_z;
	private_node_handle->param(configuration_namespace + "box_translation_x", box_translation_x, 0.0);
	private_node_handle->param(configuration_namespace + "box_translation_y", box_translation_y, 0.0);
	private_node_handle->param(configuration_namespace + "box_translation_z", box_translation_z, 0.0);
	filter->setTranslation(Eigen::Vector3f(box_translation_x, box_translation_y, box_translation_z));

	double box_rotation_roll, box_rotation_pitch, box_rotation_yaw;
	private_node_handle->param(configuration_namespace + "box_rotation_roll", box_rotation_roll, 0.0);
	private_node_handle->param(configuration_namespace + "box_rotation_pitch", box_rotation_pitch, 0.0);
	private_node_handle->param(configuration_namespace + "box_rotation_yaw", box_rotation_yaw, 0.0);
	filter->setRotation(Eigen::Vector3f(box_rotation_roll, box_rotation_pitch, box_rotation_yaw));

	bool invert_selection;
	private_node_handle->param(configuration_namespace + "invert_selection", invert_selection, false);
	filter->setNegative(invert_selection);

	CloudFilter<PointT>::setFilter(filter_base);
	CloudFilter<PointT>::setupConfigurationFromParameterServer(node_handle, private_node_handle, configuration_namespace);
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </RadiusOutlierRemoval-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

} /* namespace dynamic_robot_localization */
