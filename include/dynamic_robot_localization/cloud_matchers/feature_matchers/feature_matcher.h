#pragma once

/**\file feature_matcher.h
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

// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// external libs includes
#include <boost/smart_ptr/shared_ptr.hpp>

// project includes
#include <dynamic_robot_localization/common/pointcloud_conversions.h>
#include <dynamic_robot_localization/cloud_matchers/cloud_matcher.h>
#include <dynamic_robot_localization/cloud_matchers/feature_matchers/keypoint_descriptors/keypoint_descriptor.h>
#include <dynamic_robot_localization/cloud_matchers/feature_matchers/keypoint_descriptors/fpfh.h>
#include <dynamic_robot_localization/cloud_matchers/feature_matchers/keypoint_descriptors/shot.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {
// ############################################################################   feature_matcher   ############################################################################
/**
 * \brief Description...
 */
template <typename PointT, typename FeatureT>
class FeatureMatcher : public CloudMatcher<PointT> {
	// ========================================================================   <public-section>   ===========================================================================
	public:
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <typedefs>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		typedef boost::shared_ptr< FeatureMatcher<PointT, FeatureT> > Ptr;
		typedef boost::shared_ptr< const FeatureMatcher<PointT, FeatureT> > ConstPtr;
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </typedefs>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <enums>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </enums>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constants>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constants>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		FeatureMatcher() : save_descriptors_in_binary_format_(true) {}
		virtual ~FeatureMatcher() {}
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <FeatureMatcher-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		virtual void setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace = "");
		virtual void setupReferenceCloud(typename pcl::PointCloud<PointT>::Ptr& reference_cloud, typename pcl::PointCloud<PointT>::Ptr& reference_cloud_keypoints,
				typename pcl::search::KdTree<PointT>::Ptr& search_method);
		virtual void initializeKeypointProcessing();
		virtual void processKeypoints(typename pcl::PointCloud<PointT>::Ptr& pointcloud_keypoints,
				typename pcl::PointCloud<PointT>::Ptr& surface,
				typename pcl::search::KdTree<PointT>::Ptr& surface_search_method);

		virtual void setMatcherReferenceDescriptors(typename pcl::PointCloud<FeatureT>::Ptr& reference_descriptors) = 0;
		virtual void setMatcherAmbientDescriptors(typename pcl::PointCloud<FeatureT>::Ptr& ambient_descriptors) = 0;
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </FeatureMatcher-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <gets>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		const typename KeypointDescriptor<PointT, FeatureT>::Ptr getKeypointDescriptor() { return keypoint_descriptor_; }
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </gets>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <sets>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		void setKeypointDescriptor(const typename KeypointDescriptor<PointT, FeatureT>::Ptr& keypoint_descriptor) { keypoint_descriptor_ = keypoint_descriptor; }
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </sets>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// ========================================================================   </public-section>  ===========================================================================

	// ========================================================================   <protected-section>   ========================================================================
	protected:
		typename KeypointDescriptor<PointT, FeatureT>::Ptr keypoint_descriptor_;
		std::string reference_pointcloud_descriptors_filename_;
		std::string reference_pointcloud_descriptors_save_filename_;
		bool save_descriptors_in_binary_format_;
	// ========================================================================   </protected-section>  ========================================================================
};

} /* namespace dynamic_robot_localization */



#ifdef DRL_NO_PRECOMPILE
#include <dynamic_robot_localization/cloud_matchers/feature_matchers/impl/feature_matcher.hpp>
#endif

