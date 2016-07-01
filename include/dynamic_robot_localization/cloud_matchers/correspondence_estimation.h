#pragma once

/**\file correspondence_estimation.h
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// std includes

// ROS includes

// PCL includes
#include <pcl/common/time.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_estimation_backprojection.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/correspondence_estimation_organized_projection.h>

// external libs includes
#include <boost/smart_ptr/shared_ptr.hpp>

// project includes

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

enum CorrepondenceEstimationApproach {
	CorrespondenceEstimation,
	CorrespondenceEstimationBackProjection,
	CorrespondenceEstimationNormalShooting,
	CorrespondenceEstimationOrganizedProjection
};


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <macros>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#define UNPACK_ARGS( ... ) __VA_ARGS__
#define GenerateCorrespondenceEstimationTimed(BaseClass, Suffix, TemlatesDeclaration, TemplatesUsage) \
template < UNPACK_ARGS TemlatesDeclaration > \
class BaseClass##Suffix : public pcl::registration::BaseClass< UNPACK_ARGS TemplatesUsage > { \
	public: \
		typedef boost::shared_ptr< BaseClass##Suffix< UNPACK_ARGS TemplatesUsage > > Ptr; \
		typedef boost::shared_ptr< const BaseClass##Suffix< UNPACK_ARGS TemplatesUsage > > ConstPtr; \
\
		BaseClass##Suffix() : correspondence_estimation_elapsed_time_(0) {} \
		virtual ~BaseClass##Suffix() {} \
\
		virtual void determineCorrespondences(pcl::Correspondences &correspondences, double max_distance = std::numeric_limits<double>::max()) { \
			pcl::StopWatch timer_; \
			timer_.reset(); \
			pcl::registration::BaseClass< UNPACK_ARGS TemplatesUsage >::determineCorrespondences(correspondences, max_distance); \
			correspondence_estimation_elapsed_time_ += timer_.getTime(); \
		} \
\
		virtual void determineReciprocalCorrespondences(pcl::Correspondences &correspondences, double max_distance = std::numeric_limits<double>::max()) { \
			pcl::StopWatch timer_; \
			timer_.reset(); \
			pcl::registration::BaseClass< UNPACK_ARGS TemplatesUsage >::determineReciprocalCorrespondences(correspondences, max_distance); \
			correspondence_estimation_elapsed_time_ += timer_.getTime(); \
		} \
\
		double getCorrespondenceEstimationElapsedTime() { return correspondence_estimation_elapsed_time_; } \
		void resetCorrespondenceEstimationElapsedTime() { correspondence_estimation_elapsed_time_ = 0; } \
\
	protected: \
		double correspondence_estimation_elapsed_time_; \
};

GenerateCorrespondenceEstimationTimed(CorrespondenceEstimation, Timed, (typename PointSource, typename PointTarget, typename Scalar = float), (PointSource, PointTarget, Scalar))
GenerateCorrespondenceEstimationTimed(CorrespondenceEstimationBackProjection, Timed, (typename PointSource, typename PointTarget, typename NormalT, typename Scalar = float), (PointSource, PointTarget, NormalT, Scalar))
GenerateCorrespondenceEstimationTimed(CorrespondenceEstimationNormalShooting, Timed, (typename PointSource, typename PointTarget, typename NormalT, typename Scalar = float), (PointSource, PointTarget, NormalT, Scalar))
GenerateCorrespondenceEstimationTimed(CorrespondenceEstimationOrganizedProjection, Timed, (typename PointSource, typename PointTarget, typename Scalar = float), (PointSource, PointTarget, Scalar))
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </macros>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


} /* namespace dynamic_robot_localization */

