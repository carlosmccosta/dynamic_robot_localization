#pragma once

/**\file correspondence_estimation.h
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// std includes
#include <limits>

// ROS includes

// PCL includes
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_estimation_lookup_table.h>
#include <pcl/registration/correspondence_estimation_backprojection.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/correspondence_estimation_organized_projection.h>

// external libs includes
#include <boost/smart_ptr/shared_ptr.hpp>

// project includes
#include <dynamic_robot_localization/common/common.h>
#include <dynamic_robot_localization/common/performance_timer.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

enum CorrepondenceEstimationApproach {
	CorrespondenceEstimation,
	CorrespondenceEstimationLookupTable,
	CorrespondenceEstimationBackProjection,
	CorrespondenceEstimationNormalShooting,
	CorrespondenceEstimationOrganizedProjection
};


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <macros>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#define GenerateCorrespondenceEstimationTimed(BaseClass, Suffix, TemplatesDeclaration, TemplatesUsage) \
template < DRL_UNPACK_ARGS TemplatesDeclaration > \
class BaseClass##Suffix : public pcl::registration::BaseClass< DRL_UNPACK_ARGS TemplatesUsage > { \
	public: \
		typedef boost::shared_ptr< BaseClass##Suffix< DRL_UNPACK_ARGS TemplatesUsage > > Ptr; \
		typedef boost::shared_ptr< const BaseClass##Suffix< DRL_UNPACK_ARGS TemplatesUsage > > ConstPtr; \
\
		BaseClass##Suffix() : correspondence_estimation_elapsed_time_(0) {} \
		virtual ~BaseClass##Suffix() {} \
\
		virtual void determineCorrespondences(pcl::Correspondences &correspondences, double max_distance = std::numeric_limits<double>::max()) { \
			PerformanceTimer timer_; \
			timer_.start(); \
			pcl::registration::BaseClass< DRL_UNPACK_ARGS TemplatesUsage >::determineCorrespondences(correspondences, max_distance); \
			correspondence_estimation_elapsed_time_ += timer_.getElapsedTimeInMilliSec(); \
		} \
\
		virtual void determineReciprocalCorrespondences(pcl::Correspondences &correspondences, double max_distance = std::numeric_limits<double>::max()) { \
			PerformanceTimer timer_; \
			timer_.start(); \
			pcl::registration::BaseClass< DRL_UNPACK_ARGS TemplatesUsage >::determineReciprocalCorrespondences(correspondences, max_distance); \
			correspondence_estimation_elapsed_time_ += timer_.getElapsedTimeInMilliSec(); \
		} \
\
		inline double getCorrespondenceEstimationElapsedTime() { return correspondence_estimation_elapsed_time_; } \
		inline void resetCorrespondenceEstimationElapsedTime() { correspondence_estimation_elapsed_time_ = 0; } \
\
	protected: \
		double correspondence_estimation_elapsed_time_; \
};

GenerateCorrespondenceEstimationTimed(CorrespondenceEstimation, Timed, (typename PointSource, typename PointTarget, typename Scalar = float), (PointSource, PointTarget, Scalar))
GenerateCorrespondenceEstimationTimed(CorrespondenceEstimationLookupTable, Timed, (typename PointSource, typename PointTarget, typename Scalar = float), (PointSource, PointTarget, Scalar))
GenerateCorrespondenceEstimationTimed(CorrespondenceEstimationBackProjection, Timed, (typename PointSource, typename PointTarget, typename NormalT, typename Scalar = float), (PointSource, PointTarget, NormalT, Scalar))
GenerateCorrespondenceEstimationTimed(CorrespondenceEstimationNormalShooting, Timed, (typename PointSource, typename PointTarget, typename NormalT, typename Scalar = float), (PointSource, PointTarget, NormalT, Scalar))
GenerateCorrespondenceEstimationTimed(CorrespondenceEstimationOrganizedProjection, Timed, (typename PointSource, typename PointTarget, typename Scalar = float), (PointSource, PointTarget, Scalar))
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </macros>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


} /* namespace dynamic_robot_localization */

