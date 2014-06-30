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
void IntrinsicShapeSignature3D<PointT>::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle) {
	double salient_radius;
	private_node_handle->param("salient_radius", salient_radius, 0.06);
	keypoint_detector_.setSalientRadius(salient_radius);

	double non_max_radius;
	private_node_handle->param("non_max_radius", non_max_radius, 0.04);
	keypoint_detector_.setNonMaxRadius(non_max_radius);

	double normal_radius;
	private_node_handle->param("normal_radius", normal_radius, 0.04);
	keypoint_detector_.setNormalRadius(normal_radius);

	double border_radius;
	private_node_handle->param("border_radius", border_radius, 0.0);
	keypoint_detector_.setBorderRadius(border_radius);

	double threshold21;
	private_node_handle->param("threshold21", threshold21, 0.975);
	keypoint_detector_.setThreshold21(threshold21);

	double threshold32;
	private_node_handle->param("threshold32", threshold32, 0.975);
	keypoint_detector_.setThreshold32(threshold32);

	int min_neighbors;
	private_node_handle->param("min_neighbors", min_neighbors, 5);
	keypoint_detector_.setMinNeighbors(min_neighbors);

	double angle_threshold;
	private_node_handle->param("angle_threshold", angle_threshold, 1.57);
	keypoint_detector_.setAngleThreshold(angle_threshold);

	typename CloudPublisher<PointT>::Ptr cloud_publisher(new CloudPublisher<PointT>());
	cloud_publisher->setParameterServerArgumentToLoadTopicName("iss3_keypoints_cloud_publish_topic");
	cloud_publisher->setupConfigurationFromParameterServer(node_handle, private_node_handle);
	KeypointDetector<PointT>::setCloudPublisher(cloud_publisher);
}


template<typename PointT>
void IntrinsicShapeSignature3D<PointT>::findKeypoints(typename pcl::PointCloud<PointT>::Ptr& pointcloud, typename pcl::PointCloud<PointT>::Ptr& pointcloud_keypoints_out,
        typename pcl::PointCloud<PointT>::Ptr& surface, typename pcl::search::KdTree<PointT>::Ptr& surface_search_method) {
	keypoint_detector_.setSearchMethod(surface_search_method);
	keypoint_detector_.setSearchSurface(surface);
	keypoint_detector_.setInputCloud(pointcloud);
	keypoint_detector_.setNormals(surface);
	keypoint_detector_.compute(*pointcloud_keypoints_out);

	KeypointDetector<PointT>::getCloudPublisher()->publishPointCloud(*pointcloud_keypoints_out);
	ROS_DEBUG_STREAM("IntrinsicShapeSignature3D found " << pointcloud_keypoints_out->points.size() << " keypoints in pointcloud with " << pointcloud->points.size() << " points");
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </IntrinsicShapeSignature3D-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// =============================================================================   <private-section>   =========================================================================
// =============================================================================   </private-section>  =========================================================================

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <template instantiations>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </template instantiations>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

} /* namespace dynamic_robot_localization */

