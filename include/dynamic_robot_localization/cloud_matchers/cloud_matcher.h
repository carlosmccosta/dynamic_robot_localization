#pragma once

/**\file cloud_matcher.h
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

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>


// PCL includes
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/filters/filter.h>
#include <pcl/pcl_macros.h>
//#include <pcl/visualization/registration_visualizer.h>

// external libs includes
#include <boost/smart_ptr/shared_ptr.hpp>
#include <Eigen/Core>

// project includes
#include <dynamic_robot_localization/common/cumulative_static_transform_broadcaster.h>
#include <dynamic_robot_localization/common/configurable_object.h>
#include <dynamic_robot_localization/common/cloud_publisher.h>
#include <dynamic_robot_localization/common/math_utils.h>
#include <dynamic_robot_localization/common/performance_timer.h>
#include <dynamic_robot_localization/common/registration_visualizer.h>
#include <dynamic_robot_localization/cloud_matchers/correspondence_estimation.h>
#include <dynamic_robot_localization/cloud_matchers/transformation_estimation.h>
#include <laserscan_to_pointcloud/tf_rosmsg_eigen_conversions.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {
// #############################################################################   cloud_matcher   #############################################################################
/**
 * \brief Description...
 */
template <typename PointT>
class CloudMatcher : public ConfigurableObject {
	// ========================================================================   <public-section>   ===========================================================================
	public:
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <typedefs>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		typedef boost::shared_ptr< CloudMatcher<PointT> > Ptr;
		typedef boost::shared_ptr< const CloudMatcher<PointT> > ConstPtr;
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </typedefs>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <enums>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </enums>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constants>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constants>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		CloudMatcher();
		virtual ~CloudMatcher() {}
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <CloudMatcher-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		virtual void setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace = "");
		virtual void setupReferenceCloud(typename pcl::PointCloud<PointT>::Ptr& reference_cloud, typename pcl::PointCloud<PointT>::Ptr& reference_cloud_keypoints,
				typename pcl::search::KdTree<PointT>::Ptr& search_method);

		virtual bool registerCloud(typename pcl::PointCloud<PointT>::Ptr& ambient_pointcloud,
				typename pcl::search::KdTree<PointT>::Ptr& ambient_pointcloud_search_method,
				typename pcl::PointCloud<PointT>::Ptr& pointcloud_keypoints,
				tf2::Transform& best_pose_correction_out, std::vector< tf2::Transform >& accepted_pose_corrections_out, typename pcl::PointCloud<PointT>::Ptr& pointcloud_registered_out, bool return_aligned_keypoints = false);

		virtual void initializeKeypointProcessing() {}
		virtual void processKeypoints(typename pcl::PointCloud<PointT>::Ptr& pointcloud_keypoints,
				typename pcl::PointCloud<PointT>::Ptr& surface,
				typename pcl::search::KdTree<PointT>::Ptr& surface_search_method) {}

		virtual boost::shared_ptr< std::vector< typename pcl::Registration<PointT, PointT>::Matrix4> > getAcceptedTransformations() { return boost::shared_ptr< std::vector< typename pcl::Registration<PointT, PointT>::Matrix4> >(new std::vector< typename pcl::Registration<PointT, PointT>::Matrix4>()); }

		void setupRegistrationVisualizer();
		virtual bool registrationRequiresNormalsOnAmbientPointCloud() { return false; }
		virtual double getCorrespondenceEstimationElapsedTimeMS();
		virtual void resetCorrespondenceEstimationElapsedTime();
		virtual double getTransformationEstimationElapsedTimeMS();
		virtual void resetTransformationEstimationElapsedTime();
		virtual double getTransformCloudElapsedTimeMS() { return -1.0; }
		virtual void resetTransformCloudElapsedTime() {}
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </CloudMatcher-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <gets>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		inline typename pcl::Registration<PointT, PointT>::Ptr getCloudMatcher() { return cloud_matcher_; }
		inline bool getMatchOnlyKeypoints() const { return match_only_keypoints_; }
		inline typename CloudPublisher<PointT>::Ptr getCloudPublisher() { return cloud_publisher_; }
		inline bool getDisplayCloudAligment() const { return display_cloud_aligment_; }
		inline const boost::shared_ptr<RegistrationVisualizer<PointT, PointT> >& getRegistrationVisualizer() const { return registration_visualizer_; }
		inline double getCloudAlignTimeMS() { return cloud_align_time_ms_; }
		virtual int getNumberOfRegistrationIterations() { return -1; }
		virtual std::string getMatcherConvergenceState() { return ""; }
		virtual double getRootMeanSquareErrorOfRegistrationCorrespondences() { return -1.0; }
		virtual int getNumberCorrespondencesInLastRegistrationIteration() { return -1; }
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </gets>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <sets>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		inline void setCloudMatcher(const typename pcl::Registration<PointT, PointT>::Ptr& cloud_matcher) { cloud_matcher_ = cloud_matcher; }
		inline void setMatchOnlyKeypoints(bool match_only_keypoints) { match_only_keypoints_ = match_only_keypoints; }
		inline void setCloudPublisher(typename CloudPublisher<PointT>::Ptr& cloud_publisher) { cloud_publisher_ = cloud_publisher; }
		inline void setDisplayCloudAligment(bool display_cloud_aligment) { display_cloud_aligment_ = display_cloud_aligment; }
		inline void setForceNoRecomputeReciprocal (bool force_no_recompute_reciprocal) { force_no_recompute_reciprocal_ = force_no_recompute_reciprocal; }
		inline void setRegistrationVisualizer(const boost::shared_ptr<RegistrationVisualizer<PointT, PointT> >& registration_visualizer) { registration_visualizer_ = registration_visualizer; }
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </sets>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// ========================================================================   </public-section>  ===========================================================================

	// ========================================================================   <protected-section>   ========================================================================
	protected:
		typename pcl::Registration<PointT, PointT>::Ptr cloud_matcher_;
		CorrepondenceEstimationApproach correpondence_estimation_approach_;
		typename pcl::registration::CorrespondenceEstimationBase<PointT, PointT, float>::Ptr correspondence_estimation_ptr_;
		TransformationEstimationApproach transformation_estimation_approach_;
		typename pcl::registration::TransformationEstimation<PointT, PointT, float>::Ptr transformation_estimation_ptr_;
		double cloud_align_time_ms_;
		typename CloudPublisher<PointT>::Ptr cloud_publisher_;
		typename CloudPublisher<PointT>::Ptr reference_cloud_publisher_;
		bool match_only_keypoints_;
		typename pcl::PointCloud<PointT>::Ptr reference_cloud_;
		typename pcl::PointCloud<PointT>::Ptr reference_cloud_keypoints_;
		typename pcl::search::KdTree<PointT>::Ptr search_method_;

		boost::shared_ptr< RegistrationVisualizer<PointT, PointT> > registration_visualizer_;
		bool display_cloud_aligment_;
		int maximum_number_of_displayed_correspondences_;
		bool force_no_recompute_reciprocal_;

		boost::shared_ptr< tf2_ros::TransformBroadcaster > tf_broadcaster_;
		boost::shared_ptr< CumulativeStaticTransformBroadcaster > static_tf_broadcaster_;
		std::string tf_broadcaster_frame_id_;
		std::string tf_broadcaster_child_frame_id_;
		bool update_registered_pointcloud_with_tf_broadcaster_child_frame_id_;
	// ========================================================================   </private-section>  ==========================================================================
};

} /* namespace dynamic_robot_localization */



#ifdef DRL_NO_PRECOMPILE
#include <dynamic_robot_localization/cloud_matchers/impl/cloud_matcher.hpp>
#endif

