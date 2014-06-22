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

// ROS includes
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <dynamic_reconfigure/server.h>

// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

// external libs includes
#include <boost/smart_ptr/shared_ptr.hpp>

// project includes
#include <dynamic_robot_localization/common/configurable_object.h>
#include <dynamic_robot_localization/common/pointcloud_conversions.h>
#include <dynamic_robot_localization/localization_publisher/pose_to_tf_publisher.h>
#include <laserscan_to_pointcloud/tf_collector.h>
#include <laserscan_to_pointcloud/tf_rosmsg_eigen_conversions.h>

#include <dynamic_robot_localization/cloud_filters/cloud_filter.h>
#include <dynamic_robot_localization/cloud_filters/voxel_grid.h>

#include <dynamic_robot_localization/cloud_matchers/cloud_matcher.h>
#include <dynamic_robot_localization/cloud_matchers/point_matchers/iterative_closest_point.h>

#include <dynamic_robot_localization/transformation_validators/transformation_validator.h>
#include <dynamic_robot_localization/transformation_validators/euclidean_transformation_validator.h>

#include <dynamic_robot_localization/outlier_detectors/outlier_detector.h>
#include <dynamic_robot_localization/outlier_detectors/euclidean_outlier_detector.h>
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
		virtual void setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle);
		virtual void setupSubcriptionTopicNamesFromParameterServer();
		virtual void setupPublishTopicNamesFromParameterServer();
		virtual void setupGeneralConfigurations();

		virtual void setupFiltersConfigurations();
		virtual void setupMatchersConfigurations();
		virtual void setupTransformationValidatorsConfigurations();
		virtual void setupOutlierDetectorsConfigurations();

		bool loadReferencePointCloudFromFile(const std::string& reference_pointcloud_filename);
		void loadReferencePointCloudFromROSPointCloud(const sensor_msgs::PointCloud2ConstPtr& reference_pointcloud_msg);
		void loadReferencePointCloudFromROSOccupancyGrid(const nav_msgs::OccupancyGridConstPtr& occupancy_grid_msg);
		void publishReferencePointCloud();
		void updateLocalizationPipelineWithNewReferenceCloud();

		void startLocalization();

		void processAmbientPointCloud(const sensor_msgs::PointCloud2ConstPtr& ambient_cloud_msg);
		void resetPointCloudHeight(pcl::PointCloud<PointT>& pointcloud, float height = 0.0f);

		virtual bool updateLocalizationWithAmbientPointCloud(typename pcl::PointCloud<PointT>::Ptr& pointcloud, const tf2::Transform& pointcloud_pose_initial_guess, tf2::Transform& pointcloud_pose_corrected_out);
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
		std::string costmap_topic_;

		// publish topic names
		std::string reference_pointcloud_publish_topic_;
		std::string aligned_pointcloud_publish_topic_;
		std::string pose_publish_topic_;

		// configuration fields
		std::string reference_pointcloud_file_name_;
		std::string map_frame_id_;
		std::string base_link_frame_id_;
		ros::Duration max_seconds_scan_age_;
		ros::Duration min_seconds_between_scan_registration_;
		ros::Duration min_seconds_between_reference_pointcloud_update_;
		bool publish_tf_map_odom_;
		bool add_odometry_displacement_;

		// state fields
		ros::Time last_scan_time_;
		ros::Time last_map_received_time_;
		bool reference_pointcloud_received_;
		bool reference_pointcloud_2d_;


		// ros communication fields
		PoseToTFPublisher pose_to_tf_publisher_;
		ros::NodeHandlePtr node_handle_;
		ros::NodeHandlePtr private_node_handle_;
		ros::Subscriber ambient_pointcloud_subscriber_;
		ros::Subscriber costmap_subscriber_;
		ros::Subscriber reference_pointcloud_subscriber_;
		ros::Publisher reference_pointcloud_publisher_;
		ros::Publisher aligned_pointcloud_publisher_;
		ros::Publisher pose_publisher_;

		// localization fields
		typename pcl::PointCloud<PointT>::Ptr reference_pointcloud_;
		typename pcl::search::KdTree<PointT>::Ptr reference_pointcloud_search_method_;
		std::vector< typename CloudFilter<PointT>::Ptr > cloud_filters_;
		std::vector< typename CloudMatcher<PointT>::Ptr > cloud_matchers_;
		std::vector< TransformationValidator::Ptr > transformation_validators_;
		std::vector< typename OutlierDetector<PointT>::Ptr > outlier_detectors_;
	// ========================================================================   </private-section>  ==========================================================================
};

} /* namespace dynamic_robot_localization */


#ifdef DRL_NO_PRECOMPILE
#include <dynamic_robot_localization/localization/impl/localization.hpp>
#endif
