#pragma once

/**\file iterative_closest_point.h
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
#include <limits>
#include <algorithm>

// ROS includes

// PCL includes
#include <pcl/registration/icp.h>

// external libs includes
#include <boost/smart_ptr/shared_ptr.hpp>

// project includes
#include <dynamic_robot_localization/cloud_matchers/cloud_matcher.h>
#include <dynamic_robot_localization/common/performance_timer.h>
#include <dynamic_robot_localization/convergence_estimators/default_convergence_criteria_with_time.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

// ###############################################################   iterative_closest_point_time_constrained   ################################################################
template <typename PointSource, typename PointTarget, typename Scalar = float>
class IterativeClosestPointTimeConstrained: public pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar> {
	public:
		typedef boost::shared_ptr< IterativeClosestPointTimeConstrained<PointSource, PointTarget, Scalar> > Ptr;
		typedef boost::shared_ptr< const IterativeClosestPointTimeConstrained<PointSource, PointTarget, Scalar> > ConstPtr;

		IterativeClosestPointTimeConstrained(double convergence_time_limit_seconds = std::numeric_limits<double>::max()) {
			pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar>::convergence_criteria_.reset(new DefaultConvergenceCriteriaWithTime<Scalar> (
					pcl::Registration<PointSource, PointTarget, Scalar>::nr_iterations_,
					pcl::Registration<PointSource, PointTarget, Scalar>::transformation_,
					*pcl::Registration<PointSource, PointTarget, Scalar>::correspondences_,
					convergence_time_limit_seconds));
		}

		virtual ~IterativeClosestPointTimeConstrained() {}

		inline bool getSourceHasNormals() { return pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar>::source_has_normals_; }
		inline bool getTargetHasNormals() { return pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar>::target_has_normals_; }

		inline void setSourceHasNormals(bool source_has_normals) { pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar>::source_has_normals_ = source_has_normals; }
		inline void setTargetHasNormals(bool target_has_normals) { pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar>::target_has_normals_ = target_has_normals; }

		inline double getTransformCloudElapsedTime() { return transform_cloud_elapsed_time_ms_; }
		inline void resetTransformCloudElapsedTime() { transform_cloud_elapsed_time_ms_ = 0; }

	protected:
		virtual void transformCloud(const typename pcl::Registration<PointSource, PointTarget, Scalar>::PointCloudSource &input, typename pcl::Registration<PointSource, PointTarget, Scalar>::PointCloudSource &output, const typename pcl::Registration<PointSource, PointTarget, Scalar>::Matrix4 &transform) {
			PerformanceTimer timer_;
			timer_.start();
			pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar>::transformCloud(input, output, transform);
			transform_cloud_elapsed_time_ms_ += timer_.getElapsedTimeInMilliSec();
		}

		double transform_cloud_elapsed_time_ms_;
};


// ########################################################################   iterative_closest_point   ########################################################################
/**
 * \brief Description...
 */
template <typename PointT>
class IterativeClosestPoint : public CloudMatcher<PointT> {
	// ========================================================================   <public-section>   ===========================================================================
	public:
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <typedefs>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		typedef boost::shared_ptr< IterativeClosestPoint<PointT> > Ptr;
		typedef boost::shared_ptr< const IterativeClosestPoint<PointT> > ConstPtr;
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </typedefs>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <enums>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </enums>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constants>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constants>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		IterativeClosestPoint() : cumulative_sum_of_convergence_time_(0.0), number_of_convergence_time_measurements(0) { }
		virtual ~IterativeClosestPoint() {}
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <IterativeClosestPoint-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		virtual void setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace = "");
		virtual bool registerCloud(typename pcl::PointCloud<PointT>::Ptr& ambient_pointcloud,
						typename pcl::search::KdTree<PointT>::Ptr& ambient_pointcloud_search_method,
						typename pcl::PointCloud<PointT>::Ptr& pointcloud_keypoints,
						tf2::Transform& best_pose_correction_out, std::vector< tf2::Transform >& accepted_pose_corrections_out, typename pcl::PointCloud<PointT>::Ptr& pointcloud_registered_out, bool return_aligned_keypoints = false);
		virtual int getNumberOfRegistrationIterations();
		virtual double getRootMeanSquareErrorOfRegistrationCorrespondences();
		virtual int getNumberCorrespondencesInLastRegistrationIteration();
		typename DefaultConvergenceCriteriaWithTime<float>::Ptr getConvergenceCriteria();
		virtual std::string getMatcherConvergenceState();
		virtual double getTransformCloudElapsedTimeMS();
		virtual void resetTransformCloudElapsedTime();
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </IterativeClosestPoint-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <gets>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </gets>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <sets>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </sets>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// ========================================================================   </public-section>  ===========================================================================

	// ========================================================================   <protected-section>   ========================================================================
	protected:
		double convergence_absolute_mse_threshold_;
		double convergence_rotation_threshold_;
		int convergence_max_iterations_similar_transforms_;
		double convergence_time_limit_seconds_;
		double cumulative_sum_of_convergence_time_;
		size_t number_of_convergence_time_measurements;
		double convergence_time_limit_seconds_as_mean_convergence_time_percentage_;
		int minimum_number_of_convergence_time_measurements_to_adjust_convergence_time_limit_;
	// ========================================================================   </protected-section>  ========================================================================
};

} /* namespace dynamic_robot_localization */


#ifdef DRL_NO_PRECOMPILE
#include <dynamic_robot_localization/cloud_matchers/point_matchers/impl/iterative_closest_point.hpp>
#endif

