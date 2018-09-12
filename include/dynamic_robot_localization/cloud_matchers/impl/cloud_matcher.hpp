/**\file cloud_matcher.hpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/cloud_matchers/cloud_matcher.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
CloudMatcher<PointT>::CloudMatcher() :
		match_only_keypoints_(false),
		display_cloud_aligment_(false),
		maximum_number_of_displayed_correspondences_(0),
		cloud_align_time_ms_(0),
		force_no_recompute_reciprocal_(true) {}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <CloudMatcher-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
void CloudMatcher<PointT>::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace) {
	std::string final_param_name;
	std::string search_namespace = private_node_handle->getNamespace() + "/" + configuration_namespace;
	if (ros::param::search(search_namespace, "match_only_keypoints", final_param_name)) { private_node_handle->param(final_param_name, match_only_keypoints_, false); }
	if (ros::param::search(search_namespace, "display_cloud_aligment", final_param_name)) { private_node_handle->param(final_param_name, display_cloud_aligment_, false); }
	if (ros::param::search(search_namespace, "maximum_number_of_displayed_correspondences", final_param_name)) { private_node_handle->param(final_param_name, maximum_number_of_displayed_correspondences_, 0); } // show all

	bool publish_tf = false;
	bool publish_static_tf = false;
	if (ros::param::search(search_namespace, "tf_publisher/publish_tf", final_param_name)) { private_node_handle->param(final_param_name, publish_tf, false); }
	if (ros::param::search(search_namespace, "tf_publisher/publish_static_tf", final_param_name)) { private_node_handle->param(final_param_name, publish_static_tf, false); }

	if (ros::param::search(search_namespace, "tf_publisher/tf_broadcaster_frame_id", final_param_name)) { private_node_handle->param(final_param_name, tf_broadcaster_frame_id_, std::string("")); }
	if (ros::param::search(search_namespace, "tf_publisher/tf_broadcaster_child_frame_id", final_param_name)) { private_node_handle->param(final_param_name, tf_broadcaster_child_frame_id_, std::string("")); }

	if (ros::param::search(search_namespace, "tf_publisher/update_registered_pointcloud_with_tf_broadcaster_child_frame_id", final_param_name)) { private_node_handle->param(final_param_name, update_registered_pointcloud_with_tf_broadcaster_child_frame_id_, false); }

	if (!tf_broadcaster_frame_id_.empty()) {
		if (publish_tf)
			tf_broadcaster_ = boost::shared_ptr< tf2_ros::TransformBroadcaster >(new tf2_ros::TransformBroadcaster());

		if (publish_static_tf) {
			static_tf_broadcaster_ = CumulativeStaticTransformBroadcaster::getSingleton(node_handle);
		}

	}

	cloud_publisher_ = typename CloudPublisher<PointT>::Ptr(new CloudPublisher<PointT>());
	cloud_publisher_->setParameterServerArgumentToLoadTopicName(configuration_namespace + "registered_cloud_publish_topic");
	cloud_publisher_->setupConfigurationFromParameterServer(node_handle, private_node_handle, configuration_namespace);

	reference_cloud_publisher_ = typename CloudPublisher<PointT>::Ptr(new CloudPublisher<PointT>());
	reference_cloud_publisher_->setOverrideCloudPublishStamp(true);
	reference_cloud_publisher_->setParameterServerArgumentToLoadTopicName(configuration_namespace + "reference_cloud_publish_topic");
	reference_cloud_publisher_->setupConfigurationFromParameterServer(node_handle, private_node_handle, configuration_namespace);

	// subclass must set cloud_matcher_ ptr
	if (cloud_matcher_) {
		double max_correspondence_distance = 0.1;
		double transformation_epsilon = 1e-8;
		double euclidean_fitness_epsilon = 1e-6;
		int max_number_of_registration_iterations = 250;
		int max_number_of_ransac_iterations = 250;
		double ransac_outlier_rejection_threshold = 0.05;

		if (ros::param::search(search_namespace, "max_correspondence_distance", final_param_name)) { private_node_handle->param(final_param_name, max_correspondence_distance, 0.1); }
		if (ros::param::search(search_namespace, "transformation_epsilon", final_param_name)) { private_node_handle->param(final_param_name, transformation_epsilon, 1e-8); }
		if (ros::param::search(search_namespace, "euclidean_fitness_epsilon", final_param_name)) { private_node_handle->param(final_param_name, euclidean_fitness_epsilon, 1e-6); }
		if (ros::param::search(search_namespace, "max_number_of_registration_iterations", final_param_name)) { private_node_handle->param(final_param_name, max_number_of_registration_iterations, 250); }
		if (ros::param::search(search_namespace, "max_number_of_ransac_iterations", final_param_name)) { private_node_handle->param(final_param_name, max_number_of_ransac_iterations, 250); }
		if (ros::param::search(search_namespace, "ransac_outlier_rejection_threshold", final_param_name)) { private_node_handle->param(final_param_name, ransac_outlier_rejection_threshold, 0.05); }

		cloud_matcher_->setMaxCorrespondenceDistance(max_correspondence_distance);
		cloud_matcher_->setTransformationEpsilon(transformation_epsilon);
		cloud_matcher_->setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
		cloud_matcher_->setMaximumIterations(max_number_of_registration_iterations);
		cloud_matcher_->setRANSACIterations(max_number_of_ransac_iterations);
		cloud_matcher_->setRANSACOutlierRejectionThreshold(ransac_outlier_rejection_threshold);

		if (max_number_of_ransac_iterations > 0) {
			typename pcl::registration::CorrespondenceRejectorSampleConsensus<PointT>::Ptr rej_sac(new pcl::registration::CorrespondenceRejectorSampleConsensus<PointT>());
			rej_sac->setMaximumIterations(max_number_of_ransac_iterations);
			rej_sac->setInlierThreshold(ransac_outlier_rejection_threshold);
			cloud_matcher_->addCorrespondenceRejector(rej_sac);
		}


		correspondence_estimation_ptr_.reset();
		int correspondence_estimation_k = 0;
		if (ros::param::search(search_namespace, "correspondence_estimation_k", final_param_name)) { private_node_handle->param(final_param_name, correspondence_estimation_k, 0); }

		double correspondence_estimation_normals_angle_filtering_threshold = 80.0;
		if (ros::param::search(search_namespace, "correspondence_estimation_normals_angle_filtering_threshold", final_param_name)) { private_node_handle->param(final_param_name, correspondence_estimation_normals_angle_filtering_threshold, 80.0); }

		double correspondence_estimation_normals_angle_penalty_factor = 4.0;
		if (ros::param::search(search_namespace, "correspondence_estimation_normals_angle_penalty_factor", final_param_name)) { private_node_handle->param(final_param_name, correspondence_estimation_normals_angle_penalty_factor, 4.0); }

		std::string correspondence_estimation_method;
		if (ros::param::search(search_namespace, "correspondence_estimation_approach", final_param_name)) { private_node_handle->param(final_param_name, correspondence_estimation_method, std::string("")); }

		if (correspondence_estimation_method == "CorrespondenceEstimation") {
			correpondence_estimation_approach_ = CorrespondenceEstimation;
			correspondence_estimation_ptr_ = typename pcl::registration::CorrespondenceEstimationBase<PointT, PointT, float>::Ptr(new CorrespondenceEstimationTimed<PointT, PointT, float>());
		} else if(correspondence_estimation_method == "CorrespondenceEstimationLookupTable") {
			correpondence_estimation_approach_ = CorrespondenceEstimationLookupTable;
			CorrespondenceEstimationLookupTableTimed<PointT, PointT, float>* correspondence_estimation_raw_ptr_ = new CorrespondenceEstimationLookupTableTimed<PointT, PointT, float>();
			double map_cell_resolution = 0.01, map_margin_x = 1.0, map_margin_y = 1.0, map_margin_z = 1.0;
			bool map_use_search_tree_when_query_point_is_outside_lookup_table = true;
			bool map_compute_distance_from_query_point_to_closest_point = false;
			bool map_initialize_lookup_table_using_euclidean_distance_transform = true;
			double sensor_cell_resolution = 0.01, sensor_margin_x = 1.0, sensor_margin_y = 1.0, sensor_margin_z = 1.0;
			bool sensor_use_search_tree_when_query_point_is_outside_lookup_table = true;
			bool sensor_compute_distance_from_query_point_to_closest_point = false;
			bool sensor_initialize_lookup_table_using_euclidean_distance_transform = true;
			if (ros::param::search(search_namespace, "correspondence_estimation_lookup_table/map_cell_resolution", final_param_name)) { private_node_handle->param(final_param_name, map_cell_resolution, 0.01); }
			if (ros::param::search(search_namespace, "correspondence_estimation_lookup_table/map_margin_x", final_param_name)) { private_node_handle->param(final_param_name, map_margin_x, 1.0); }
			if (ros::param::search(search_namespace, "correspondence_estimation_lookup_table/map_margin_y", final_param_name)) { private_node_handle->param(final_param_name, map_margin_y, 1.0); }
			if (ros::param::search(search_namespace, "correspondence_estimation_lookup_table/map_margin_z", final_param_name)) { private_node_handle->param(final_param_name, map_margin_z, 1.0); }
			if (ros::param::search(search_namespace, "correspondence_estimation_lookup_table/map_use_search_tree_when_query_point_is_outside_lookup_table", final_param_name)) { private_node_handle->param(final_param_name, map_use_search_tree_when_query_point_is_outside_lookup_table, true); }
			if (ros::param::search(search_namespace, "correspondence_estimation_lookup_table/map_compute_distance_from_query_point_to_closest_point", final_param_name)) { private_node_handle->param(final_param_name, map_compute_distance_from_query_point_to_closest_point, false); }
			if (ros::param::search(search_namespace, "correspondence_estimation_lookup_table/map_initialize_lookup_table_using_euclidean_distance_transform", final_param_name)) { private_node_handle->param(final_param_name, map_initialize_lookup_table_using_euclidean_distance_transform, true); }
			if (ros::param::search(search_namespace, "correspondence_estimation_lookup_table/sensor_cell_resolution", final_param_name)) { private_node_handle->param(final_param_name, sensor_cell_resolution, 0.01); }
			if (ros::param::search(search_namespace, "correspondence_estimation_lookup_table/sensor_margin_x", final_param_name)) { private_node_handle->param(final_param_name, sensor_margin_x, 1.0); }
			if (ros::param::search(search_namespace, "correspondence_estimation_lookup_table/sensor_margin_y", final_param_name)) { private_node_handle->param(final_param_name, sensor_margin_y, 1.0); }
			if (ros::param::search(search_namespace, "correspondence_estimation_lookup_table/sensor_margin_z", final_param_name)) { private_node_handle->param(final_param_name, sensor_margin_z, 1.0); }
			if (ros::param::search(search_namespace, "correspondence_estimation_lookup_table/sensor_use_search_tree_when_query_point_is_outside_lookup_table", final_param_name)) { private_node_handle->param(final_param_name, sensor_use_search_tree_when_query_point_is_outside_lookup_table, true); }
			if (ros::param::search(search_namespace, "correspondence_estimation_lookup_table/sensor_compute_distance_from_query_point_to_closest_point", final_param_name)) { private_node_handle->param(final_param_name, sensor_compute_distance_from_query_point_to_closest_point, false); }
			if (ros::param::search(search_namespace, "correspondence_estimation_lookup_table/sensor_initialize_lookup_table_using_euclidean_distance_transform", final_param_name)) { private_node_handle->param(final_param_name, sensor_initialize_lookup_table_using_euclidean_distance_transform, true); }
			correspondence_estimation_raw_ptr_->getTargetCorrespondencesLookupTable().setCellResolution(map_cell_resolution);
			correspondence_estimation_raw_ptr_->getTargetCorrespondencesLookupTable().setLookupTableMargin(Eigen::Vector3f(map_margin_x, map_margin_y, map_margin_z));
			correspondence_estimation_raw_ptr_->getTargetCorrespondencesLookupTable().setUseSearchTreeWhenQueryPointIsOutsideLookupTable(map_use_search_tree_when_query_point_is_outside_lookup_table);
			correspondence_estimation_raw_ptr_->getTargetCorrespondencesLookupTable().setComputeDistanceFromQueryPointToClosestPoint(map_compute_distance_from_query_point_to_closest_point);
			correspondence_estimation_raw_ptr_->getTargetCorrespondencesLookupTable().setInitializeLookupTableUsingEuclideanDistanceTransform(map_initialize_lookup_table_using_euclidean_distance_transform);
			correspondence_estimation_raw_ptr_->getSourceCorrespondencesLookupTable().setCellResolution(sensor_cell_resolution);
			correspondence_estimation_raw_ptr_->getSourceCorrespondencesLookupTable().setLookupTableMargin(Eigen::Vector3f(sensor_margin_x, sensor_margin_y, sensor_margin_z));
			correspondence_estimation_raw_ptr_->getSourceCorrespondencesLookupTable().setUseSearchTreeWhenQueryPointIsOutsideLookupTable(sensor_use_search_tree_when_query_point_is_outside_lookup_table);
			correspondence_estimation_raw_ptr_->getSourceCorrespondencesLookupTable().setComputeDistanceFromQueryPointToClosestPoint(sensor_compute_distance_from_query_point_to_closest_point);
			correspondence_estimation_raw_ptr_->getSourceCorrespondencesLookupTable().setInitializeLookupTableUsingEuclideanDistanceTransform(sensor_initialize_lookup_table_using_euclidean_distance_transform);
			correspondence_estimation_ptr_ = typename pcl::registration::CorrespondenceEstimationBase<PointT, PointT, float>::Ptr(correspondence_estimation_raw_ptr_);
		} else if (correspondence_estimation_method == "CorrespondenceEstimationBackProjection") {
			correpondence_estimation_approach_ = CorrespondenceEstimationBackProjection;
			CorrespondenceEstimationBackProjectionTimed<PointT, PointT, PointT, float>* correspondence_estimation_raw_ptr_ = new CorrespondenceEstimationBackProjectionTimed<PointT, PointT, PointT, float>();
			correspondence_estimation_raw_ptr_->setKSearch(correspondence_estimation_k);
			correspondence_estimation_raw_ptr_->setNormalsAngleFilteringThreshold(correspondence_estimation_normals_angle_filtering_threshold);
			correspondence_estimation_raw_ptr_->setNormalsAnglePenaltyFactor(correspondence_estimation_normals_angle_penalty_factor);
			correspondence_estimation_ptr_ = typename pcl::registration::CorrespondenceEstimationBase<PointT, PointT, float>::Ptr(correspondence_estimation_raw_ptr_);
		} else if (correspondence_estimation_method == "CorrespondenceEstimationNormalShooting") {
			correpondence_estimation_approach_ = CorrespondenceEstimationNormalShooting;
			CorrespondenceEstimationNormalShootingTimed<PointT, PointT, PointT, float>* correspondence_estimation_raw_ptr_ = new CorrespondenceEstimationNormalShootingTimed<PointT, PointT, PointT, float>();
			correspondence_estimation_raw_ptr_->setKSearch(correspondence_estimation_k);
			correspondence_estimation_ptr_ = typename pcl::registration::CorrespondenceEstimationBase<PointT, PointT, float>::Ptr(correspondence_estimation_raw_ptr_);
		} else if (correspondence_estimation_method == "CorrespondenceEstimationOrganizedProjection") {
			correpondence_estimation_approach_ = CorrespondenceEstimationOrganizedProjection;
			CorrespondenceEstimationOrganizedProjectionTimed<PointT, PointT, float>* correspondence_estimation_raw_ptr_ = new CorrespondenceEstimationOrganizedProjectionTimed<PointT, PointT, float>();
			double fx = 525.0, fy = 525.0, cx = 319.5, cy = 239.5;
			if (ros::param::search(search_namespace, "correspondence_estimation_organized_projection/fx", final_param_name)) { private_node_handle->param(final_param_name, fx, 525.0); }
			if (ros::param::search(search_namespace, "correspondence_estimation_organized_projection/fy", final_param_name)) { private_node_handle->param(final_param_name, fy, 525.0); }
			if (ros::param::search(search_namespace, "correspondence_estimation_organized_projection/cx", final_param_name)) { private_node_handle->param(final_param_name, cx, 319.5); }
			if (ros::param::search(search_namespace, "correspondence_estimation_organized_projection/cy", final_param_name)) { private_node_handle->param(final_param_name, cy, 239.5); }
			correspondence_estimation_raw_ptr_->setFocalLengths(fx, fy);
			correspondence_estimation_raw_ptr_->setCameraCenters(cx, cy);
			correspondence_estimation_ptr_ = typename pcl::registration::CorrespondenceEstimationBase<PointT, PointT, float>::Ptr(correspondence_estimation_raw_ptr_);
		}

		if (correspondence_estimation_ptr_) {
			cloud_matcher_->setCorrespondenceEstimation(correspondence_estimation_ptr_);
		}


		std::string transformation_estimation_method;
		if (ros::param::search(search_namespace, "transformation_estimation_approach", final_param_name)) { private_node_handle->param(final_param_name, transformation_estimation_method, std::string("")); }
		if (transformation_estimation_method == "TransformationEstimation2D") {
			transformation_estimation_approach_ = TransformationEstimation2D;
			transformation_estimation_ptr_ = typename pcl::registration::TransformationEstimation<PointT, PointT, float>::Ptr(new TransformationEstimation2DTimed<PointT, PointT, float>());
		} else if (transformation_estimation_method == "TransformationEstimationDualQuaternion") {
			transformation_estimation_approach_ = TransformationEstimationDualQuaternion;
			transformation_estimation_ptr_ = typename pcl::registration::TransformationEstimation<PointT, PointT, float>::Ptr(new TransformationEstimationDualQuaternionTimed<PointT, PointT, float>());
		} else if (transformation_estimation_method == "TransformationEstimationLM") {
			transformation_estimation_approach_ = TransformationEstimationLM;
			transformation_estimation_ptr_ = typename pcl::registration::TransformationEstimation<PointT, PointT, float>::Ptr(new TransformationEstimationLMTimed<PointT, PointT, float>());
		} else if (transformation_estimation_method == "TransformationEstimationPointToPlane") {
			transformation_estimation_approach_ = TransformationEstimationPointToPlane;
			transformation_estimation_ptr_ = typename pcl::registration::TransformationEstimation<PointT, PointT, float>::Ptr(new TransformationEstimationPointToPlaneTimed<PointT, PointT, float>());
		} else if (transformation_estimation_method == "TransformationEstimationPointToPlaneLLS") {
			transformation_estimation_approach_ = TransformationEstimationPointToPlaneLLS;
			transformation_estimation_ptr_ = typename pcl::registration::TransformationEstimation<PointT, PointT, float>::Ptr(new TransformationEstimationPointToPlaneLLSTimed<PointT, PointT, float>());
		} else if (transformation_estimation_method == "TransformationEstimationPointToPlaneLLSWeighted") {
			transformation_estimation_approach_ = TransformationEstimationPointToPlaneLLSWeighted;
			transformation_estimation_ptr_ = typename pcl::registration::TransformationEstimation<PointT, PointT, float>::Ptr(new TransformationEstimationPointToPlaneLLSWeightedTimed<PointT, PointT, float>());
		} else if (transformation_estimation_method == "TransformationEstimationPointToPlaneWeighted") {
			transformation_estimation_approach_ = TransformationEstimationPointToPlaneWeighted;
			transformation_estimation_ptr_ = typename pcl::registration::TransformationEstimation<PointT, PointT, float>::Ptr(new TransformationEstimationPointToPlaneWeightedTimed<PointT, PointT, float>());
		} else if (transformation_estimation_method == "TransformationEstimationSVD") {
			transformation_estimation_approach_ = TransformationEstimationSVD;
			transformation_estimation_ptr_ = typename pcl::registration::TransformationEstimation<PointT, PointT, float>::Ptr(new TransformationEstimationSVDTimed<PointT, PointT, float>());
		} else if (transformation_estimation_method == "TransformationEstimationSVDScale") {
			transformation_estimation_approach_ = TransformationEstimationSVDScale;
			transformation_estimation_ptr_ = typename pcl::registration::TransformationEstimation<PointT, PointT, float>::Ptr(new TransformationEstimationSVDScaleTimed<PointT, PointT, float>());
		}

		if (transformation_estimation_ptr_) {
			cloud_matcher_->setTransformationEstimation(transformation_estimation_ptr_);
		}


		setupRegistrationVisualizer();
	}
}


template<typename PointT>
void CloudMatcher<PointT>::setupReferenceCloud(typename pcl::PointCloud<PointT>::Ptr& reference_cloud, typename pcl::PointCloud<PointT>::Ptr& reference_cloud_keypoints,
		typename pcl::search::KdTree<PointT>::Ptr& search_method) {

	reference_cloud_ = reference_cloud;
	reference_cloud_keypoints_ = reference_cloud_keypoints;
	search_method_ = search_method;

	// subclass must set cloud_matcher_ ptr
	if (cloud_matcher_) {
		cloud_matcher_->setInputTarget(reference_cloud);
		cloud_matcher_->setSearchMethodTarget(search_method, true);
		if (cloud_matcher_->getCorrespondenceEstimation())
			cloud_matcher_->getCorrespondenceEstimation()->setSearchMethodTarget(search_method, false);
	}

	if (registration_visualizer_) {
		registration_visualizer_->setTargetCloud(*reference_cloud);
	}
}


template<typename PointT>
bool CloudMatcher<PointT>::registerCloud(typename pcl::PointCloud<PointT>::Ptr& ambient_pointcloud,
		typename pcl::search::KdTree<PointT>::Ptr& ambient_pointcloud_search_method,
		typename pcl::PointCloud<PointT>::Ptr& pointcloud_keypoints,
		tf2::Transform& best_pose_correction_out, std::vector< tf2::Transform >& accepted_pose_corrections_out, typename pcl::PointCloud<PointT>::Ptr& pointcloud_registered_out, bool return_aligned_keypoints) {

	// subclass must set cloud_matcher_ ptr
	if (!cloud_matcher_) {
		return false;
	}
	std::vector<int> indexes;
	pcl::removeNaNFromPointCloud(*ambient_pointcloud, *ambient_pointcloud, indexes);
	indexes.clear();
	pcl::removeNaNNormalsFromPointCloud(*ambient_pointcloud, *ambient_pointcloud, indexes);
	indexes.clear();

	if (ambient_pointcloud->size() < 3) {
		ROS_WARN("Discarded ambient cloud with less than 3 points before performing registration.");
		return false;
	}

	if (ambient_pointcloud->size() != ambient_pointcloud_search_method->getInputCloud()->size()) {
		ambient_pointcloud_search_method->setInputCloud(ambient_pointcloud);
	}

	initializeKeypointProcessing();

	if (match_only_keypoints_ && !pointcloud_keypoints->empty()) {
		ROS_DEBUG_STREAM("Registering cloud with " << pointcloud_keypoints->size() << " keypoints against a reference cloud with " << cloud_matcher_->getInputTarget()->size() << " points using " << cloud_matcher_->getClassName() << " algorithm");
		typename pcl::search::KdTree<PointT>::Ptr pointcloud_keypoints_search_method(new pcl::search::KdTree<PointT>());
		pointcloud_keypoints_search_method->setInputCloud(pointcloud_keypoints);
		cloud_matcher_->setInputSource(pointcloud_keypoints);
		cloud_matcher_->setSearchMethodSource(pointcloud_keypoints_search_method, force_no_recompute_reciprocal_);
		if (registration_visualizer_) { registration_visualizer_->setSourceCloud(*pointcloud_keypoints); }
	} else {
		ROS_DEBUG_STREAM("Registering cloud with " << ambient_pointcloud->size() << " points against a reference cloud with " << cloud_matcher_->getInputTarget()->size() << " points using " << cloud_matcher_->getClassName() << " algorithm");
		cloud_matcher_->setInputSource(ambient_pointcloud);
		cloud_matcher_->setSearchMethodSource(ambient_pointcloud_search_method, force_no_recompute_reciprocal_);
		if (registration_visualizer_) { registration_visualizer_->setSourceCloud(*ambient_pointcloud); }
	}

	processKeypoints(pointcloud_keypoints, ambient_pointcloud, ambient_pointcloud_search_method);

	resetCorrespondenceEstimationElapsedTime();
	resetTransformationEstimationElapsedTime();
	resetTransformCloudElapsedTime();
	cloud_align_time_ms_ = 0;
	PerformanceTimer performance_timer;
	performance_timer.start();
	cloud_matcher_->align(*pointcloud_registered_out);
	cloud_align_time_ms_ = performance_timer.getElapsedTimeInMilliSec();

	Eigen::Matrix4f final_transformation = cloud_matcher_->getFinalTransformation();

	std::string final_transform_str = math_utils::convertTransformToString(final_transformation, "\n\t\t[ ", " ]", " | ");
	ROS_DEBUG_STREAM("Cloud matcher [" << cloud_matcher_->getClassName() << "] transform:" << final_transform_str << "\n");

	if (!math_utils::isTransformValid<float>(final_transformation)) {
		ROS_WARN("Rejected estimated transformation with NaN values!");
		return false; // a transform with NaNs will cause a crash because of kd-tree search
	}

	laserscan_to_pointcloud::tf_rosmsg_eigen_conversions::transformMatrixToTF2(final_transformation, best_pose_correction_out);

	if (tf_broadcaster_ || static_tf_broadcaster_) {
		geometry_msgs::TransformStamped final_transformation_msg;
		laserscan_to_pointcloud::tf_rosmsg_eigen_conversions::transformTF2ToMsg(best_pose_correction_out.inverse(), final_transformation_msg.transform);
		pcl_conversions::fromPCL(ambient_pointcloud->header.stamp, final_transformation_msg.header.stamp);
		final_transformation_msg.header.frame_id = tf_broadcaster_frame_id_;
		final_transformation_msg.child_frame_id = tf_broadcaster_child_frame_id_;

		if (update_registered_pointcloud_with_tf_broadcaster_child_frame_id_)
			ambient_pointcloud->header.frame_id = tf_broadcaster_child_frame_id_;

		if (tf_broadcaster_)
			tf_broadcaster_->sendTransform(final_transformation_msg);

		if (static_tf_broadcaster_)
			static_tf_broadcaster_->sendTransform(final_transformation_msg);
	}


	if (cloud_matcher_->hasConverged()) {
		boost::shared_ptr< std::vector< typename pcl::Registration<PointT, PointT>::Matrix4 > > acceptedTransformations = getAcceptedTransformations();

		if (!acceptedTransformations->empty()) {
			for (size_t i = 0; i < acceptedTransformations->size(); ++i) {
				tf2::Transform transform;
				laserscan_to_pointcloud::tf_rosmsg_eigen_conversions::transformMatrixToTF2((*acceptedTransformations)[i], transform);
				accepted_pose_corrections_out.push_back(transform);
			}

			ROS_INFO_STREAM("Initial pose estimation found " << acceptedTransformations->size() << " acceptable poses");
			acceptedTransformations->clear();
		}

		if (return_aligned_keypoints && !match_only_keypoints_) {
			pcl::transformPointCloudWithNormals(*pointcloud_keypoints, *pointcloud_registered_out, final_transformation);
		} else if (!return_aligned_keypoints && match_only_keypoints_) {
			pcl::transformPointCloudWithNormals(*ambient_pointcloud, *pointcloud_registered_out, final_transformation);
		}

		if (pointcloud_registered_out->size() < 5) {
			return false;
		}

		if (pointcloud_keypoints && !pointcloud_keypoints->empty()) {
			pcl::transformPointCloudWithNormals(*pointcloud_keypoints, *pointcloud_keypoints, final_transformation);
		}

		pointcloud_registered_out->header = ambient_pointcloud->header;
		pointcloud_keypoints->header = ambient_pointcloud->header;

		// if publisher available, send aligned cloud
		if (cloud_publisher_ && pointcloud_registered_out) {
			cloud_publisher_->publishPointCloud(*pointcloud_registered_out);
		}

		if (reference_cloud_publisher_ && reference_cloud_) {
			reference_cloud_publisher_->setCloudPublishStamp(ambient_pointcloud->header.stamp);
			reference_cloud_publisher_->publishPointCloud(*reference_cloud_);
		}

		return true;
	}

	return false;
}


template<typename PointT>
void CloudMatcher<PointT>::setupRegistrationVisualizer() {
	if (cloud_matcher_ && !registration_visualizer_ && display_cloud_aligment_) {
		registration_visualizer_ = boost::shared_ptr< RegistrationVisualizer<PointT, PointT> >(new RegistrationVisualizer<PointT, PointT>());
		registration_visualizer_->setMaximumDisplayedCorrespondences(maximum_number_of_displayed_correspondences_);
		registration_visualizer_->setRegistration(*cloud_matcher_);

		registration_visualizer_->startDisplay();
		ROS_DEBUG_STREAM("RegistrationVisualizer activated with " << maximum_number_of_displayed_correspondences_ << " number_maximum_displayed_correspondences");
	}
}

template<typename PointT>
double CloudMatcher<PointT>::getCorrespondenceEstimationElapsedTimeMS() {
	if (correspondence_estimation_ptr_) {
		switch (correpondence_estimation_approach_) {
			case CorrespondenceEstimation: {
				typename CorrespondenceEstimationTimed<PointT, PointT, float>::Ptr estimator = boost::dynamic_pointer_cast< CorrespondenceEstimationTimed<PointT, PointT, float> >(correspondence_estimation_ptr_);
				if (estimator) { return estimator->getCorrespondenceEstimationElapsedTime(); }
				break;
			}

			case CorrespondenceEstimationLookupTable: {
				typename CorrespondenceEstimationLookupTableTimed<PointT, PointT, float>::Ptr estimator = boost::dynamic_pointer_cast< CorrespondenceEstimationLookupTableTimed<PointT, PointT, float> >(correspondence_estimation_ptr_);
				if (estimator) { return estimator->getCorrespondenceEstimationElapsedTime(); }
				break;
			}

			case CorrespondenceEstimationBackProjection: {
				typename CorrespondenceEstimationBackProjectionTimed<PointT, PointT, PointT, float>::Ptr estimator = boost::dynamic_pointer_cast< CorrespondenceEstimationBackProjectionTimed<PointT, PointT, PointT, float> >(correspondence_estimation_ptr_);
				if (estimator) { return estimator->getCorrespondenceEstimationElapsedTime(); }
				break;
			}

			case CorrespondenceEstimationNormalShooting: {
				typename CorrespondenceEstimationNormalShootingTimed<PointT, PointT, PointT, float>::Ptr estimator = boost::dynamic_pointer_cast< CorrespondenceEstimationNormalShootingTimed<PointT, PointT, PointT, float> >(correspondence_estimation_ptr_);
				if (estimator) { return estimator->getCorrespondenceEstimationElapsedTime(); }
				break;
			}

			case CorrespondenceEstimationOrganizedProjection: {
				typename CorrespondenceEstimationOrganizedProjectionTimed<PointT, PointT, float>::Ptr estimator = boost::dynamic_pointer_cast< CorrespondenceEstimationOrganizedProjectionTimed<PointT, PointT, float> >(correspondence_estimation_ptr_);
				if (estimator) { return estimator->getCorrespondenceEstimationElapsedTime(); }
				break;
			}

			default:
				break;
		}
	}

	return -1.0;
}

template<typename PointT>
void CloudMatcher<PointT>::resetCorrespondenceEstimationElapsedTime() {
	if (correspondence_estimation_ptr_) {
		switch (correpondence_estimation_approach_) {
			case CorrespondenceEstimation: {
				typename CorrespondenceEstimationTimed<PointT, PointT, float>::Ptr estimator = boost::dynamic_pointer_cast< CorrespondenceEstimationTimed<PointT, PointT, float> >(correspondence_estimation_ptr_);
				if (estimator) { estimator->resetCorrespondenceEstimationElapsedTime(); }
				break;
			}

			case CorrespondenceEstimationLookupTable: {
				typename CorrespondenceEstimationLookupTableTimed<PointT, PointT, float>::Ptr estimator = boost::dynamic_pointer_cast< CorrespondenceEstimationLookupTableTimed<PointT, PointT, float> >(correspondence_estimation_ptr_);
				if (estimator) { estimator->resetCorrespondenceEstimationElapsedTime(); }
				break;
			}

			case CorrespondenceEstimationBackProjection: {
				typename CorrespondenceEstimationBackProjectionTimed<PointT, PointT, PointT, float>::Ptr estimator = boost::dynamic_pointer_cast< CorrespondenceEstimationBackProjectionTimed<PointT, PointT, PointT, float> >(correspondence_estimation_ptr_);
				if (estimator) { estimator->resetCorrespondenceEstimationElapsedTime(); }
				break;
			}

			case CorrespondenceEstimationNormalShooting: {
				typename CorrespondenceEstimationNormalShootingTimed<PointT, PointT, PointT, float>::Ptr estimator = boost::dynamic_pointer_cast< CorrespondenceEstimationNormalShootingTimed<PointT, PointT, PointT, float> >(correspondence_estimation_ptr_);
				if (estimator) { estimator->resetCorrespondenceEstimationElapsedTime(); }
				break;
			}

			case CorrespondenceEstimationOrganizedProjection: {
				typename CorrespondenceEstimationOrganizedProjectionTimed<PointT, PointT, float>::Ptr estimator = boost::dynamic_pointer_cast< CorrespondenceEstimationOrganizedProjectionTimed<PointT, PointT, float> >(correspondence_estimation_ptr_);
				if (estimator) { estimator->resetCorrespondenceEstimationElapsedTime(); }
				break;
			}

			default:
				break;
		}
	}
}

template<typename PointT>
double CloudMatcher<PointT>::getTransformationEstimationElapsedTimeMS() {
	if (transformation_estimation_ptr_) {
		switch (transformation_estimation_approach_) {
			case TransformationEstimation2D: {
				typename TransformationEstimation2DTimed<PointT, PointT, float>::Ptr estimator = boost::dynamic_pointer_cast< TransformationEstimation2DTimed<PointT, PointT, float> >(transformation_estimation_ptr_);
				if (estimator) { return estimator->getTransformationEstimationElapsedTime(); }
				break;
			}

			case TransformationEstimationDualQuaternion: {
				typename TransformationEstimationDualQuaternionTimed<PointT, PointT, float>::Ptr estimator = boost::dynamic_pointer_cast< TransformationEstimationDualQuaternionTimed<PointT, PointT, float> >(transformation_estimation_ptr_);
				if (estimator) { return estimator->getTransformationEstimationElapsedTime(); }
				break;
			}

			case TransformationEstimationLM: {
				typename TransformationEstimationLMTimed<PointT, PointT, float>::Ptr estimator = boost::dynamic_pointer_cast< TransformationEstimationLMTimed<PointT, PointT, float> >(transformation_estimation_ptr_);
				if (estimator) { return estimator->getTransformationEstimationElapsedTime(); }
				break;
			}

			case TransformationEstimationPointToPlane: {
				typename TransformationEstimationPointToPlaneTimed<PointT, PointT, float>::Ptr estimator = boost::dynamic_pointer_cast< TransformationEstimationPointToPlaneTimed<PointT, PointT, float> >(transformation_estimation_ptr_);
				if (estimator) { return estimator->getTransformationEstimationElapsedTime(); }
				break;
			}

			case TransformationEstimationPointToPlaneLLS: {
				typename TransformationEstimationPointToPlaneLLSTimed<PointT, PointT, float>::Ptr estimator = boost::dynamic_pointer_cast< TransformationEstimationPointToPlaneLLSTimed<PointT, PointT, float> >(transformation_estimation_ptr_);
				if (estimator) { return estimator->getTransformationEstimationElapsedTime(); }
				break;
			}

			case TransformationEstimationPointToPlaneLLSWeighted: {
				typename TransformationEstimationPointToPlaneLLSWeightedTimed<PointT, PointT, float>::Ptr estimator = boost::dynamic_pointer_cast< TransformationEstimationPointToPlaneLLSWeightedTimed<PointT, PointT, float> >(transformation_estimation_ptr_);
				if (estimator) { return estimator->getTransformationEstimationElapsedTime(); }
				break;
			}

			case TransformationEstimationPointToPlaneWeighted: {
				typename TransformationEstimationPointToPlaneWeightedTimed<PointT, PointT, float>::Ptr estimator = boost::dynamic_pointer_cast< TransformationEstimationPointToPlaneWeightedTimed<PointT, PointT, float> >(transformation_estimation_ptr_);
				if (estimator) { return estimator->getTransformationEstimationElapsedTime(); }
				break;
			}

			case TransformationEstimationSVD: {
				typename TransformationEstimationSVDTimed<PointT, PointT, float>::Ptr estimator = boost::dynamic_pointer_cast< TransformationEstimationSVDTimed<PointT, PointT, float> >(transformation_estimation_ptr_);
				if (estimator) { return estimator->getTransformationEstimationElapsedTime(); }
				break;
			}

			case TransformationEstimationSVDScale: {
				typename TransformationEstimationSVDScaleTimed<PointT, PointT, float>::Ptr estimator = boost::dynamic_pointer_cast< TransformationEstimationSVDScaleTimed<PointT, PointT, float> >(transformation_estimation_ptr_);
				if (estimator) { return estimator->getTransformationEstimationElapsedTime(); }
				break;
			}

			default:
				break;
		}
	}

	return -1.0;
}

template<typename PointT>
void CloudMatcher<PointT>::resetTransformationEstimationElapsedTime() {
	if (transformation_estimation_ptr_) {
		switch (transformation_estimation_approach_) {
			case TransformationEstimation2D: {
				typename TransformationEstimation2DTimed<PointT, PointT, float>::Ptr estimator = boost::dynamic_pointer_cast< TransformationEstimation2DTimed<PointT, PointT, float> >(transformation_estimation_ptr_);
				if (estimator) { estimator->resetTransformationEstimationElapsedTime(); }
				break;
			}

			case TransformationEstimationDualQuaternion: {
				typename TransformationEstimationDualQuaternionTimed<PointT, PointT, float>::Ptr estimator = boost::dynamic_pointer_cast< TransformationEstimationDualQuaternionTimed<PointT, PointT, float> >(transformation_estimation_ptr_);
				if (estimator) { estimator->resetTransformationEstimationElapsedTime(); }
				break;
			}

			case TransformationEstimationLM: {
				typename TransformationEstimationLMTimed<PointT, PointT, float>::Ptr estimator = boost::dynamic_pointer_cast< TransformationEstimationLMTimed<PointT, PointT, float> >(transformation_estimation_ptr_);
				if (estimator) { estimator->resetTransformationEstimationElapsedTime(); }
				break;
			}

			case TransformationEstimationPointToPlane: {
				typename TransformationEstimationPointToPlaneTimed<PointT, PointT, float>::Ptr estimator = boost::dynamic_pointer_cast< TransformationEstimationPointToPlaneTimed<PointT, PointT, float> >(transformation_estimation_ptr_);
				if (estimator) { estimator->resetTransformationEstimationElapsedTime(); }
				break;
			}

			case TransformationEstimationPointToPlaneLLS: {
				typename TransformationEstimationPointToPlaneLLSTimed<PointT, PointT, float>::Ptr estimator = boost::dynamic_pointer_cast< TransformationEstimationPointToPlaneLLSTimed<PointT, PointT, float> >(transformation_estimation_ptr_);
				if (estimator) { estimator->resetTransformationEstimationElapsedTime(); }
				break;
			}

			case TransformationEstimationPointToPlaneLLSWeighted: {
				typename TransformationEstimationPointToPlaneLLSWeightedTimed<PointT, PointT, float>::Ptr estimator = boost::dynamic_pointer_cast< TransformationEstimationPointToPlaneLLSWeightedTimed<PointT, PointT, float> >(transformation_estimation_ptr_);
				if (estimator) { estimator->resetTransformationEstimationElapsedTime(); }
				break;
			}

			case TransformationEstimationPointToPlaneWeighted: {
				typename TransformationEstimationPointToPlaneWeightedTimed<PointT, PointT, float>::Ptr estimator = boost::dynamic_pointer_cast< TransformationEstimationPointToPlaneWeightedTimed<PointT, PointT, float> >(transformation_estimation_ptr_);
				if (estimator) { estimator->resetTransformationEstimationElapsedTime(); }
				break;
			}

			case TransformationEstimationSVD: {
				typename TransformationEstimationSVDTimed<PointT, PointT, float>::Ptr estimator = boost::dynamic_pointer_cast< TransformationEstimationSVDTimed<PointT, PointT, float> >(transformation_estimation_ptr_);
				if (estimator) { estimator->resetTransformationEstimationElapsedTime(); }
				break;
			}

			case TransformationEstimationSVDScale: {
				typename TransformationEstimationSVDScaleTimed<PointT, PointT, float>::Ptr estimator = boost::dynamic_pointer_cast< TransformationEstimationSVDScaleTimed<PointT, PointT, float> >(transformation_estimation_ptr_);
				if (estimator) { estimator->resetTransformationEstimationElapsedTime(); }
				break;
			}

			default:
				break;
		}
	}
}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </CloudMatcher-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <template instantiations>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </template instantiations>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

} /* namespace dynamic_robot_localization */


