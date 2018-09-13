/**\file plane_segmentation.hpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/cloud_filters/plane_segmentation.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <PlaneSegmentation-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
void PlaneSegmentation<PointT>::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace) {
	private_node_handle->param(configuration_namespace + "sample_consensus_method", sample_consensus_method_, std::string("SAC_RANSAC"));
	private_node_handle->param(configuration_namespace + "use_surface_normals", use_surface_normals_, true);
	private_node_handle->param(configuration_namespace + "sample_consensus_maximum_distance_of_sample_to_plane", sample_consensus_maximum_distance_of_sample_to_plane_, 0.01);
	private_node_handle->param(configuration_namespace + "sample_consensus_normals_difference_weight", sample_consensus_normals_difference_weight_, 0.1);
	private_node_handle->param(configuration_namespace + "sample_consensus_number_of_iterations", sample_consensus_number_of_iterations_, 1000);
	private_node_handle->param(configuration_namespace + "sample_consensus_probability_of_sample_not_be_an_outlier", sample_consensus_probability_of_sample_not_be_an_outlier_, 0.5);
	private_node_handle->param(configuration_namespace + "plane_convex_hull_scaling_factor", plane_convex_hull_scaling_factor_, 1.0);
	private_node_handle->param(configuration_namespace + "segmentation_minimum_distance_to_plane", segmentation_minimum_distance_to_plane_, 0.01);
	private_node_handle->param(configuration_namespace + "segmentation_maximum_distance_to_plane", segmentation_maximum_distance_to_plane_, 0.42);

	plane_inliers_cloud_publisher_ = typename CloudPublisher<PointT>::Ptr(new CloudPublisher<PointT>());
	plane_inliers_cloud_publisher_->setParameterServerArgumentToLoadTopicName(configuration_namespace + "plane_inliers_cloud_publish_topic");
	plane_inliers_cloud_publisher_->setupConfigurationFromParameterServer(node_handle, private_node_handle, configuration_namespace);

	plane_inliers_convex_hull_cloud_publisher_ = typename CloudPublisher<PointT>::Ptr(new CloudPublisher<PointT>());
	plane_inliers_convex_hull_cloud_publisher_->setParameterServerArgumentToLoadTopicName(configuration_namespace + "plane_inliers_convex_hull_cloud_publish_topic");
	plane_inliers_convex_hull_cloud_publisher_->setupConfigurationFromParameterServer(node_handle, private_node_handle, configuration_namespace);

	CloudFilter<PointT>::setupConfigurationFromParameterServer(node_handle, private_node_handle, configuration_namespace);
}

template<typename PointT>
void PlaneSegmentation<PointT>::filter(const typename pcl::PointCloud<PointT>::Ptr& input_cloud, typename pcl::PointCloud<PointT>::Ptr& output_cloud) {
	size_t number_of_points_in_input_cloud = input_cloud->size();

	boost::shared_ptr< pcl::SACSegmentation<PointT> > sac_segmentation;
	boost::shared_ptr< pcl::SACSegmentationFromNormals<PointT, PointT> > sac_segmentation_from_normals;

	if (use_surface_normals_) {
		sac_segmentation_from_normals = boost::shared_ptr< pcl::SACSegmentationFromNormals<PointT, PointT> >(new pcl::SACSegmentationFromNormals<PointT, PointT>());
		sac_segmentation_from_normals->setModelType(pcl::SACMODEL_NORMAL_PLANE);
		sac_segmentation_from_normals->setNormalDistanceWeight(sample_consensus_normals_difference_weight_);
		sac_segmentation_from_normals->setInputNormals(input_cloud);
		sac_segmentation = sac_segmentation_from_normals;
	} else {
		sac_segmentation = boost::shared_ptr< pcl::SACSegmentation<PointT> >(new pcl::SACSegmentation<PointT>());
		sac_segmentation->setModelType(pcl::SACMODEL_PLANE);
	}

	sac_segmentation->setDistanceThreshold(sample_consensus_maximum_distance_of_sample_to_plane_);
	sac_segmentation->setMaxIterations(sample_consensus_number_of_iterations_);
	sac_segmentation->setProbability(sample_consensus_probability_of_sample_not_be_an_outlier_);
	sac_segmentation->setOptimizeCoefficients(true);

	if (sample_consensus_method_ == "SAC_LMEDS")
		sac_segmentation->setMethodType(pcl::SAC_LMEDS);
	else if (sample_consensus_method_ == "SAC_MSAC")
		sac_segmentation->setMethodType(pcl::SAC_MSAC);
	else if (sample_consensus_method_ == "SAC_RRANSAC")
		sac_segmentation->setMethodType(pcl::SAC_RRANSAC);
	else if (sample_consensus_method_ == "SAC_RMSAC")
		sac_segmentation->setMethodType(pcl::SAC_RMSAC);
	else if (sample_consensus_method_ == "SAC_MLESAC")
		sac_segmentation->setMethodType(pcl::SAC_MLESAC);
	else if (sample_consensus_method_ == "SAC_PROSAC")
		sac_segmentation->setMethodType(pcl::SAC_PROSAC);
	else
		sac_segmentation->setMethodType(pcl::SAC_RANSAC);

	sac_segmentation->setInputCloud(input_cloud);

	pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices());
	pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients());
	sac_segmentation->segment(*plane_inliers, *plane_coefficients);

	if (plane_inliers->indices.size() > 0) {
		if (plane_inliers_cloud_publisher_ && !plane_inliers_cloud_publisher_->getCloudPublishTopic().empty()) {
			pcl::ExtractIndices<PointT> plane_inliers_extractor;
			plane_inliers_extractor.setInputCloud(input_cloud);
			plane_inliers_extractor.setIndices(plane_inliers);
			typename pcl::PointCloud<PointT>::Ptr plane_inliers_pointcloud(new pcl::PointCloud<PointT>());
			plane_inliers_extractor.filter(*plane_inliers_pointcloud);
			plane_inliers_cloud_publisher_->publishPointCloud(*plane_inliers_pointcloud);
		}

		pcl::ProjectInliers<PointT> plane_inliers_projection_;

		if (use_surface_normals_)
			plane_inliers_projection_.setModelType(pcl::SACMODEL_NORMAL_PLANE);
		else
			plane_inliers_projection_.setModelType(pcl::SACMODEL_PLANE);

		plane_inliers_projection_.setInputCloud(input_cloud);
		plane_inliers_projection_.setIndices(plane_inliers);
		plane_inliers_projection_.setModelCoefficients(plane_coefficients);
		typename pcl::PointCloud<PointT>::Ptr plane_inliers_projected_into_plane_model(new pcl::PointCloud<PointT>());
		plane_inliers_projection_.filter (*plane_inliers_projected_into_plane_model);

		pcl::ConvexHull<PointT> plane_convex_hull_;
		plane_convex_hull_.setInputCloud(plane_inliers_projected_into_plane_model);
		typename pcl::PointCloud<PointT>::Ptr convex_hull_for_projected_plane_inliers(new pcl::PointCloud<PointT>());
		plane_convex_hull_.reconstruct(*convex_hull_for_projected_plane_inliers);

		if (plane_convex_hull_scaling_factor_ != 1.0) {
			Eigen::Vector4f centroid;
			pcl::compute3DCentroid(*convex_hull_for_projected_plane_inliers, centroid);

			Eigen::Matrix4f centroid_matrix = Eigen::Matrix4f::Identity();
			centroid_matrix.col(3).head<3>() << centroid(0), centroid(1), centroid(2);

			pcl::transformPointCloudWithNormals<PointT>(*convex_hull_for_projected_plane_inliers, *convex_hull_for_projected_plane_inliers, centroid_matrix.inverse());

			Eigen::Matrix4f scale_matrix = Eigen::Matrix4f(Eigen::Matrix4f::Identity()) * plane_convex_hull_scaling_factor_;
			pcl::transformPointCloudWithNormals<PointT>(*convex_hull_for_projected_plane_inliers, *convex_hull_for_projected_plane_inliers, scale_matrix);

			pcl::transformPointCloudWithNormals<PointT>(*convex_hull_for_projected_plane_inliers, *convex_hull_for_projected_plane_inliers, centroid_matrix);
		}

		if (plane_inliers_convex_hull_cloud_publisher_ && !plane_inliers_convex_hull_cloud_publisher_->getCloudPublishTopic().empty()) {
			plane_inliers_convex_hull_cloud_publisher_->publishPointCloud(*convex_hull_for_projected_plane_inliers);
		}

		pcl::ExtractPolygonalPrismData<PointT> polygonal_segmentation_;
		polygonal_segmentation_.setHeightLimits(segmentation_minimum_distance_to_plane_, segmentation_maximum_distance_to_plane_);
		polygonal_segmentation_.setInputCloud(input_cloud);
		polygonal_segmentation_.setInputPlanarHull(convex_hull_for_projected_plane_inliers);
		pcl::PointIndices indices_for_points_on_top_of_plane;
		polygonal_segmentation_.segment(indices_for_points_on_top_of_plane);

		pcl::ExtractIndices<PointT> indices_extractor;
		indices_extractor.setInputCloud(input_cloud);
		indices_extractor.setIndices(boost::make_shared<const pcl::PointIndices>(indices_for_points_on_top_of_plane));
		indices_extractor.filter(*output_cloud);
	}

	if (CloudFilter<PointT>::getCloudPublisher() && output_cloud) { CloudFilter<PointT>::getCloudPublisher()->publishPointCloud(*output_cloud); }
	ROS_DEBUG_STREAM(CloudFilter<PointT>::filter_name_ << " filter reduced point cloud from " << number_of_points_in_input_cloud << " points to " << output_cloud->size() << " points");
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </PlaneSegmentation-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

} /* namespace dynamic_robot_localization */
