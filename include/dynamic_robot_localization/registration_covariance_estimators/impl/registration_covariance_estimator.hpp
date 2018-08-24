/**\file registration_covariance_estimator.hpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/registration_covariance_estimators/registration_covariance_estimator.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <RegistrationCovarianceEstimator-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
void RegistrationCovarianceEstimator<PointT>::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace) {
	private_node_handle->param(configuration_namespace + "correspondence_distance_threshold", correspondence_distance_threshold_, 0.05);
	private_node_handle->param(configuration_namespace + "sensor_std_dev_noise", sensor_std_dev_noise_, 0.01);
	private_node_handle->param(configuration_namespace + "use_reciprocal_correspondences", use_reciprocal_correspondences_, false);

	int number_of_random_sampples;
	private_node_handle->param(configuration_namespace + "random_sample/number_of_random_samples", number_of_random_sampples, -1);
	if (number_of_random_sampples > 0) {
		random_sample_filter_.reset(new RandomSample<PointT>());
		random_sample_filter_->setupConfigurationFromParameterServer(node_handle, private_node_handle, configuration_namespace + "random_sample/");
	}

	std::string correspondence_estimator;
	private_node_handle->param(configuration_namespace + "correspondance_estimator", correspondence_estimator, std::string("CorrespondenceEstimation"));

	if (correspondence_estimator == "CorrespondenceEstimationBackProjection") {
		correspondence_estimation_ = typename pcl::registration::CorrespondenceEstimationBase<PointT, PointT>::Ptr(new pcl::registration::CorrespondenceEstimationBackProjection<PointT, PointT, PointT>());
	} else if (correspondence_estimator == "CorrespondenceEstimationNormalShooting") {
		correspondence_estimation_ = typename pcl::registration::CorrespondenceEstimationBase<PointT, PointT>::Ptr(new pcl::registration::CorrespondenceEstimationNormalShooting<PointT, PointT, PointT>());
	} else {
		correspondence_estimation_ = typename pcl::registration::CorrespondenceEstimationBase<PointT, PointT>::Ptr(new pcl::registration::CorrespondenceEstimation<PointT, PointT>());
	}

	cloud_publisher_reference_cloud_ = typename CloudPublisher<PointT>::Ptr(new CloudPublisher<PointT>());
	cloud_publisher_reference_cloud_->setParameterServerArgumentToLoadTopicName(configuration_namespace + "filtered_reference_cloud_publish_topic");
	cloud_publisher_reference_cloud_->setupConfigurationFromParameterServer(node_handle, private_node_handle, configuration_namespace);


	cloud_publisher_ambient_cloud_ = typename CloudPublisher<PointT>::Ptr(new CloudPublisher<PointT>());
	cloud_publisher_ambient_cloud_->setParameterServerArgumentToLoadTopicName(configuration_namespace + "filtered_ambient_cloud_publish_topic");
	cloud_publisher_ambient_cloud_->setupConfigurationFromParameterServer(node_handle, private_node_handle, configuration_namespace);
}

template<typename PointT>
void RegistrationCovarianceEstimator<PointT>::setReferenceCloud(const typename pcl::PointCloud<PointT>::Ptr& reference_cloud, const typename pcl::search::KdTree<PointT>::Ptr& reference_cloud_search_method) {
	correspondence_estimation_->setInputTarget(reference_cloud);
	if (correspondence_estimation_->requiresTargetNormals()) {
		typename pcl::PCLPointCloud2::Ptr reference_cloud_blob(new pcl::PCLPointCloud2());
		pcl::toPCLPointCloud2 (*reference_cloud, *reference_cloud_blob);
		correspondence_estimation_->setTargetNormals(reference_cloud_blob);
	}

	if (reference_cloud_search_method) {
		correspondence_estimation_->setSearchMethodTarget(reference_cloud_search_method, true);
	} else {
		correspondence_estimation_->getSearchMethodTarget()->setInputCloud(reference_cloud);
	}
}


template<typename PointT>
bool RegistrationCovarianceEstimator<PointT>::computeRegistrationCovariance(const typename pcl::PointCloud<PointT>::Ptr& cloud, const typename pcl::search::KdTree<PointT>::Ptr& search_method,
		const Eigen::Matrix4f& registration_corrections, const Eigen::Transform<float, 3, Eigen::Affine>& transform_from_map_cloud_data_to_base_link,
		const std::string& base_link_frame_id, Eigen::MatrixXd& covariance_out) {
	typename pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>());

	size_t cloud_size = cloud->size();
	if (random_sample_filter_) {
		random_sample_filter_->filter(cloud, filtered_cloud);
	} else {
		filtered_cloud = cloud;
	}

	correspondence_estimation_->setInputSource(filtered_cloud);
	if (correspondence_estimation_->requiresSourceNormals()) {
		typename pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2());
		pcl::toPCLPointCloud2(*filtered_cloud, *cloud_blob);
		correspondence_estimation_->setSourceNormals(cloud_blob);
	}

	if (use_reciprocal_correspondences_) {
		if (search_method && cloud_size == filtered_cloud->size()) {
			correspondence_estimation_->setSearchMethodSource(search_method);
		} else {
			correspondence_estimation_->getSearchMethodSource()->setInputCloud(filtered_cloud);
		}
	}

	pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
	if (use_reciprocal_correspondences_) {
		correspondence_estimation_->determineReciprocalCorrespondences (*correspondences, correspondence_distance_threshold_);
	} else {
		correspondence_estimation_->determineCorrespondences (*correspondences, correspondence_distance_threshold_);
	}

	pcl::PointCloud<PointT> reference_cloud_correspondences_map_frame;
	pcl::PointCloud<PointT> ambient_cloud_correspondences_map_frame;

	for (size_t i = 0; i < correspondences->size(); ++i) {
		reference_cloud_correspondences_map_frame.push_back((*correspondence_estimation_->getInputTarget())[(*correspondences)[i].index_match]);
		ambient_cloud_correspondences_map_frame.push_back((*correspondence_estimation_->getInputSource())[(*correspondences)[i].index_query]);
	}

	ROS_DEBUG_STREAM("Computing covariance for " << correspondences->size() << " correspondences");

	pcl::PointCloud<PointT> reference_cloud_correspondences;
	pcl::PointCloud<PointT> ambient_cloud_correspondences;

	pcl::transformPointCloudWithNormals(reference_cloud_correspondences_map_frame, reference_cloud_correspondences, transform_from_map_cloud_data_to_base_link);
	pcl::transformPointCloudWithNormals(ambient_cloud_correspondences_map_frame, ambient_cloud_correspondences, transform_from_map_cloud_data_to_base_link);

	reference_cloud_correspondences.header = cloud->header;
	ambient_cloud_correspondences.header = cloud->header;
	reference_cloud_correspondences.header.frame_id = base_link_frame_id;
	ambient_cloud_correspondences.header.frame_id = base_link_frame_id;

	if (cloud_publisher_reference_cloud_) {
		cloud_publisher_reference_cloud_->publishPointCloud(reference_cloud_correspondences);
	}

	if (cloud_publisher_ambient_cloud_) {
		cloud_publisher_ambient_cloud_->publishPointCloud(ambient_cloud_correspondences);
	}

	return computeRegistrationCovariance(reference_cloud_correspondences, ambient_cloud_correspondences, registration_corrections, covariance_out, sensor_std_dev_noise_);
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </RegistrationCovarianceEstimator-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <template instantiations>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </template instantiations>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

} /* namespace dynamic_robot_localization */
