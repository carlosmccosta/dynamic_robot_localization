#pragma once

/**\file localization.h
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <macros>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </macros>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// std includes
#include <string>
#include <vector>
#include <algorithm>
#include <utility>
#include <cmath>


// ROS includes
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <dynamic_reconfigure/server.h>
#include <angles/angles.h>

// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

// external libs includes
#include <boost/smart_ptr/shared_ptr.hpp>

// project includes
#include <dynamic_robot_localization/common/configurable_object.h>
#include <dynamic_robot_localization/common/pointcloud_conversions.h>
#include <pose_to_tf_publisher/pose_to_tf_publisher.h>
#include <laserscan_to_pointcloud/tf_collector.h>
#include <laserscan_to_pointcloud/tf_rosmsg_eigen_conversions.h>

#include <dynamic_robot_localization/cloud_filters/cloud_filter.h>
#include <dynamic_robot_localization/cloud_filters/voxel_grid.h>
#include <dynamic_robot_localization/cloud_filters/pass_through.h>

#include <dynamic_robot_localization/curvature_estimators/curvature_estimator.h>
#include <dynamic_robot_localization/curvature_estimators/principal_curvatures_estimation.h>
#include <dynamic_robot_localization/normal_estimators/normal_estimator.h>
#include <dynamic_robot_localization/normal_estimators/normal_estimator_sac.h>
#include <dynamic_robot_localization/normal_estimators/normal_estimation_omp.h>
#include <dynamic_robot_localization/normal_estimators/moving_least_squares.h>

#include <dynamic_robot_localization/cloud_matchers/feature_matchers/keypoint_detectors/keypoint_detector.h>
#include <dynamic_robot_localization/cloud_matchers/feature_matchers/keypoint_detectors/intrinsic_shape_signature_3d.h>
#include <dynamic_robot_localization/cloud_matchers/feature_matchers/keypoint_detectors/sift_3d.h>

#include <dynamic_robot_localization/cloud_matchers/feature_matchers/keypoint_descriptors/keypoint_descriptor.h>
#include <dynamic_robot_localization/cloud_matchers/feature_matchers/keypoint_descriptors/pfh.h>
#include <dynamic_robot_localization/cloud_matchers/feature_matchers/keypoint_descriptors/fpfh.h>
#include <dynamic_robot_localization/cloud_matchers/feature_matchers/keypoint_descriptors/shot.h>
#include <dynamic_robot_localization/cloud_matchers/feature_matchers/keypoint_descriptors/shape_context_3d.h>
#include <dynamic_robot_localization/cloud_matchers/feature_matchers/keypoint_descriptors/unique_shape_context.h>
#include <dynamic_robot_localization/cloud_matchers/feature_matchers/keypoint_descriptors/esf.h>

#include <dynamic_robot_localization/cloud_matchers/cloud_matcher.h>
#include <dynamic_robot_localization/cloud_matchers/point_matchers/iterative_closest_point.h>
#include <dynamic_robot_localization/cloud_matchers/point_matchers/iterative_closest_point_non_linear.h>
#include <dynamic_robot_localization/cloud_matchers/point_matchers/iterative_closest_point_with_normals.h>
#include <dynamic_robot_localization/cloud_matchers/point_matchers/iterative_closest_point_generalized.h>
#include <dynamic_robot_localization/cloud_matchers/point_matchers/normal_distributions_transform_2d.h>
#include <dynamic_robot_localization/cloud_matchers/point_matchers/normal_distributions_transform_3d.h>
#include <dynamic_robot_localization/cloud_matchers/feature_matchers/sample_consensus_initial_alignment.h>
#include <dynamic_robot_localization/cloud_matchers/feature_matchers/sample_consensus_initial_alignment_prerejective.h>

#include <dynamic_robot_localization/transformation_validators/transformation_validator.h>
#include <dynamic_robot_localization/transformation_validators/euclidean_transformation_validator.h>

#include <dynamic_robot_localization/outlier_detectors/outlier_detector.h>
#include <dynamic_robot_localization/outlier_detectors/euclidean_outlier_detector.h>

#include <dynamic_robot_localization/common/circular_buffer_pointcloud.h>
#include <dynamic_robot_localization/common/performance_timer.h>

// project msgs
#include <dynamic_robot_localization/LocalizationDetailed.h>
#include <dynamic_robot_localization/LocalizationDiagnostics.h>
#include <dynamic_robot_localization/LocalizationTimes.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {
// ##############################################################################   localization   #############################################################################
/**
 * \brief Description...
 */
template <typename PointT = pcl::PointNormal>
class Localization : public ConfigurableObject {
	// ========================================================================   <public-section>   ===========================================================================
	public:
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <typedefs>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//		typedef pcl::PointNormal PointT;
		typedef boost::shared_ptr< Localization<PointT> > Ptr;
		typedef boost::shared_ptr< const Localization<PointT> > ConstPtr;
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </typedefs>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <enums>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </enums>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constants>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constants>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		Localization();
		virtual ~Localization();
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <Localization-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		virtual void setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace = "");
		void setupGeneralConfigurations();
		void setupSubcriptionTopicNames();
		void setupPublishTopicNames();
		void setupFrameIds();
		void setupMessageManagement();
		void setupReferencePointCloud();

		virtual void setupFiltersConfigurations();
		void loadFiltersFromParameterServer(std::vector< typename CloudFilter<PointT>::Ptr >& filters_container, std::string configuration_namespace);
		virtual void setupNormalEstimatorsConfigurations();
		virtual void setupCurvatureEstimatorsConfigurations();
		void loadNormalEstimatorFromParameterServer(typename NormalEstimator<PointT>::Ptr& normal_estimator, std::string configuration_namespace);
		void loadCurvatureEstimatorFromParameterServer(typename NormalEstimator<PointT>::Ptr& normal_estimator, std::string configuration_namespace);
		virtual void setupKeypointDetectors();
		void loadKeypointDetectorsFromParameterServer(std::vector<typename KeypointDetector<PointT>::Ptr >& keypoint_detectors, std::string configuration_namespace);
		virtual void setupCloudMatchersConfigurations();
		virtual void setupPointCloudMatchersConfigurations(std::vector< typename CloudMatcher<PointT>::Ptr >& pointcloud_matchers, const std::string& configuration_namespace);
		virtual void setupFeatureCloudMatchersConfigurations(std::vector< typename CloudMatcher<PointT>::Ptr >& featurecloud_matchers, const std::string& configuration_namespace);
		template <typename DescriptorT>
		void loadKeypointMatcherFromParameterServer(std::vector< typename CloudMatcher<PointT>::Ptr >& featurecloud_matchers, typename KeypointDescriptor<PointT, DescriptorT>::Ptr& keypoint_descriptor,
				const std::string& keypoint_descriptor_configuration_namespace, const std::string& feature_matcher_configuration_namespace);
		virtual void setupTransformationValidatorsConfigurations(std::vector< TransformationValidator::Ptr >& validators, const std::string& configuration_namespace);
		virtual void setupOutlierDetectorsConfigurations();

		bool loadReferencePointCloudFromFile(const std::string& reference_pointcloud_filename);
		void loadReferencePointCloudFromROSPointCloud(const sensor_msgs::PointCloud2ConstPtr& reference_pointcloud_msg);
		void loadReferencePointCloudFromROSOccupancyGrid(const nav_msgs::OccupancyGridConstPtr& occupancy_grid_msg);
		void publishReferencePointCloud();
		bool updateLocalizationPipelineWithNewReferenceCloud();

		void startLocalization();

		void processAmbientPointCloud(const sensor_msgs::PointCloud2ConstPtr& ambient_cloud_msg);
		void resetPointCloudHeight(pcl::PointCloud<PointT>& pointcloud, float height = 0.0f);


		virtual bool applyFilters(std::vector< typename CloudFilter<PointT>::Ptr >& cloud_filters, typename pcl::PointCloud<PointT>::Ptr& pointcloud);

		virtual bool applyNormalEstimation(typename NormalEstimator<PointT>::Ptr& normal_estimator, typename pcl::PointCloud<PointT>::Ptr& pointcloud,
				typename pcl::PointCloud<PointT>::Ptr& surface,
				typename pcl::search::KdTree<PointT>::Ptr& pointcloud_search_method);

		virtual bool applyKeypointDetection(std::vector< typename KeypointDetector<PointT>::Ptr >& keypoint_detectors, typename pcl::PointCloud<PointT>::Ptr& pointcloud,
				typename pcl::search::KdTree<PointT>::Ptr& surface_search_method,
				typename pcl::PointCloud<PointT>::Ptr& keypoints);

		virtual bool applyCloudRegistration(std::vector< typename CloudMatcher<PointT>::Ptr >& matchers, typename pcl::PointCloud<PointT>::Ptr& ambient_pointcloud,
				typename pcl::search::KdTree<PointT>::Ptr& surface_search_method,
				typename pcl::PointCloud<PointT>::Ptr& pointcloud_keypoints,
				tf2::Transform& pointcloud_pose_in_out);

		virtual double applyOutlierDetection(typename pcl::PointCloud<PointT>::Ptr& ambient_pointcloud);
		virtual void publishDetectedOutliers();
		virtual void publishDetectedInliers();

		virtual bool applyTransformationValidators(std::vector< TransformationValidator::Ptr >& transformation_validators,
				const tf2::Transform& pointcloud_pose_initial_guess, tf2::Transform& pointcloud_pose_corrected_in_out, double max_outlier_percentage);

		virtual bool updateLocalizationWithAmbientPointCloud(typename pcl::PointCloud<PointT>::Ptr& pointcloud,
				const tf2::Transform& pointcloud_pose_initial_guess,
				tf2::Transform& pointcloud_pose_corrected_out);
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </Localization-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <gets>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </gets>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <sets>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </sets>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// ========================================================================   </public-section>  ===========================================================================

	// ========================================================================   <protected-section>   ========================================================================
	protected:
	// ========================================================================   </protected-section>  ========================================================================

	// ========================================================================   <private-section>   ==========================================================================
	private:
		// subscription topic names
		std::string ambient_pointcloud_topic_;
		std::string reference_pointcloud_topic_;
		std::string reference_costmap_topic_;

		// publish topic names
		std::string reference_pointcloud_publish_topic_;
		std::string aligned_pointcloud_publish_topic_;
		std::string pose_stamped_publish_topic_;
		std::string pose_with_covariance_stamped_publish_topic_;
		std::string localization_detailed_publish_topic_;
		std::string localization_diagnostics_publish_topic_;
		std::string localization_times_publish_topic_;


		// configuration fields
		std::string reference_pointcloud_filename_;
		std::string reference_pointcloud_preprocessed_save_filename_;
		std::string reference_pointcloud_keypoints_filename_;
		std::string reference_pointcloud_keypoints_save_filename_;
		std::string map_frame_id_;
		std::string base_link_frame_id_;
		std::string sensor_frame_id_;
		ros::Duration max_seconds_ambient_pointcloud_age_;
		ros::Duration min_seconds_between_scan_registration_;
		ros::Duration min_seconds_between_reference_pointcloud_update_;
		ros::Duration pose_tracking_timeout_;
		int minimum_number_of_points_in_ambient_pointcloud_;
		bool localization_detailed_use_millimeters_in_translation_corrections_;
		bool localization_detailed_use_degrees_in_rotation_corrections_;
		bool save_reference_pointclouds_in_binary_format_;
		double max_outliers_percentage_;
		bool publish_tf_map_odom_;
		bool add_odometry_displacement_;
		bool use_filtered_cloud_as_normal_estimation_surface_;
		bool compute_normals_when_tracking_pose_;
		bool compute_normals_when_recovering_pose_tracking_;
		bool compute_normals_when_estimating_initial_pose_;
		bool compute_keypoints_when_tracking_pose_;
		bool compute_keypoints_when_recovering_pose_tracking_;
		bool compute_keypoints_when_estimating_initial_pose_;
		bool compute_inliers_angular_distribution_;
		bool compute_outliers_angular_distribution_;

		// state fields
		ros::Time last_scan_time_;
		ros::Time last_map_received_time_;
		ros::Time last_accepted_pose_time_;
		int pose_tracking_maximum_number_of_failed_registrations_since_last_valid_pose_;
		int pose_tracking_minimum_number_of_failed_registrations_since_last_valid_pose_;
		int pose_tracking_number_of_failed_registrations_since_last_valid_pose_;
		bool reference_pointcloud_received_;
		bool reference_pointcloud_2d_;
		bool ignore_height_corrections_;
		bool last_accepted_pose_valid_;
		tf2::Transform last_accepted_pose_;


		// ros communication fields
		pose_to_tf_publisher::PoseToTFPublisher pose_to_tf_publisher_;
		ros::NodeHandlePtr node_handle_;
		ros::NodeHandlePtr private_node_handle_;
		ros::Subscriber ambient_pointcloud_subscriber_;
		ros::Subscriber costmap_subscriber_;
		ros::Subscriber reference_pointcloud_subscriber_;
		ros::Publisher reference_pointcloud_publisher_;
		ros::Publisher aligned_pointcloud_publisher_;
		ros::Publisher pose_with_covariance_stamped_publisher_;
		ros::Publisher pose_stamped_publisher_;
		ros::Publisher localization_detailed_publisher_;
		ros::Publisher localization_diagnostics_publisher_;
		ros::Publisher localization_times_publisher_;

		// localization fields
		typename pcl::PointCloud<PointT>::Ptr reference_pointcloud_;
		typename CircularBufferPointCloud<PointT>::Ptr ambient_pointcloud_with_circular_buffer_;
		size_t last_number_points_inserted_in_circular_buffer_;
		typename pcl::search::KdTree<PointT>::Ptr reference_pointcloud_search_method_;
		std::vector< typename CloudFilter<PointT>::Ptr > reference_cloud_filters_;
		std::vector< typename CloudFilter<PointT>::Ptr > ambient_cloud_filters_;
		typename NormalEstimator<PointT>::Ptr reference_cloud_normal_estimator_;
		typename NormalEstimator<PointT>::Ptr ambient_cloud_normal_estimator_;
		std::vector< typename KeypointDetector<PointT>::Ptr > reference_cloud_keypoint_detectors_;
		std::vector< typename KeypointDetector<PointT>::Ptr > ambient_cloud_keypoint_detectors_;
		std::vector< typename CloudMatcher<PointT>::Ptr > initial_pose_estimators_matchers_;
		std::vector< typename CloudMatcher<PointT>::Ptr > tracking_matchers_;
		std::vector< typename CloudMatcher<PointT>::Ptr > tracking_recovery_matchers_;
		std::vector< TransformationValidator::Ptr > transformation_validators_;
		std::vector< TransformationValidator::Ptr > transformation_validators_tracking_recovery_;
		std::vector< typename OutlierDetector<PointT>::Ptr > outlier_detectors_;
		double outlier_percentage_;
		size_t number_inliers_;
		double root_mean_square_error_;
		std::vector< sensor_msgs::PointCloud2Ptr > detected_outliers_;
		std::vector< sensor_msgs::PointCloud2Ptr > detected_inliers_;
		LocalizationDiagnostics localization_diagnostics_msg_;
		LocalizationTimes localization_times_msg_;
	// ========================================================================   </private-section>  ==========================================================================
};

} /* namespace dynamic_robot_localization */



#ifdef DRL_NO_PRECOMPILE
#include <dynamic_robot_localization/localization/impl/localization.hpp>
#endif
