/**\file intrinsic_shape_signature_3d.hpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/cloud_matchers/feature_matchers/keypoint_detectors/intrinsic_shape_signature_3d.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <IntrinsicShapeSignature3D-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
void IntrinsicShapeSignature3D<PointT>::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace) {
	typename pcl::Keypoint<PointT, PointT>::Ptr keypoint_detector_base(new pcl::ISSKeypoint3D<PointT, PointT, PointT>());
	typename pcl::ISSKeypoint3D<PointT, PointT, PointT>::Ptr keypoint_detector = boost::static_pointer_cast< pcl::ISSKeypoint3D<PointT, PointT, PointT> >(keypoint_detector_base);

	double salient_radius;
	private_node_handle->param(configuration_namespace + "salient_radius", salient_radius, 0.06);
	keypoint_detector->setSalientRadius(salient_radius);

	double non_max_radius;
	private_node_handle->param(configuration_namespace + "non_max_radius", non_max_radius, 0.04);
	keypoint_detector->setNonMaxRadius(non_max_radius);

	double normal_radius;
	private_node_handle->param(configuration_namespace + "normal_radius", normal_radius, 0.04);
	keypoint_detector->setNormalRadius(normal_radius);

	double border_radius;
	private_node_handle->param(configuration_namespace + "border_radius", border_radius, 0.0);
	keypoint_detector->setBorderRadius(border_radius);

	double threshold21;
	private_node_handle->param(configuration_namespace + "threshold21", threshold21, 0.975);
	keypoint_detector->setThreshold21(threshold21);

	double threshold32;
	private_node_handle->param(configuration_namespace + "threshold32", threshold32, 0.975);
	keypoint_detector->setThreshold32(threshold32);

	int min_neighbors;
	private_node_handle->param(configuration_namespace + "min_neighbors", min_neighbors, 5);
	keypoint_detector->setMinNeighbors(min_neighbors);

	double angle_threshold;
	private_node_handle->param(configuration_namespace + "angle_threshold", angle_threshold, 1.57);
	keypoint_detector->setAngleThreshold(angle_threshold);

	KeypointDetector<PointT>::setKeypointDetector(keypoint_detector_base);
	typename CloudPublisher<PointT>::Ptr cloud_publisher(new CloudPublisher<PointT>());
	cloud_publisher->setParameterServerArgumentToLoadTopicName(configuration_namespace + "iss3d_keypoints_cloud_publish_topic");
	cloud_publisher->setupConfigurationFromParameterServer(node_handle, private_node_handle, configuration_namespace);
	KeypointDetector<PointT>::setCloudPublisher(cloud_publisher);
}


template<typename PointT>
void IntrinsicShapeSignature3D<PointT>::findKeypoints(typename pcl::PointCloud<PointT>::Ptr& pointcloud, typename pcl::PointCloud<PointT>::Ptr& pointcloud_keypoints_out,
		typename pcl::PointCloud<PointT>::Ptr& surface, typename pcl::search::KdTree<PointT>::Ptr& surface_search_method) {
	boost::static_pointer_cast< pcl::ISSKeypoint3D<PointT, PointT, PointT> >(KeypointDetector<PointT>::getKeypointDetector())->setNormals(surface);

	KeypointDetector<PointT>::findKeypoints(pointcloud, pointcloud_keypoints_out, surface, surface_search_method);
	ROS_DEBUG_STREAM("IntrinsicShapeSignature3D found " << pointcloud_keypoints_out->size() << " keypoints in pointcloud with " << pointcloud->size() << " points");
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </IntrinsicShapeSignature3D-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <template instantiations>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </template instantiations>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

} /* namespace dynamic_robot_localization */

