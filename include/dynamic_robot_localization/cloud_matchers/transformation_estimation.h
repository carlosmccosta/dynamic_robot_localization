#pragma once

/**\file transformation_estimation.h
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
#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/registration/transformation_estimation_dual_quaternion.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls_weighted.h>
#include <pcl/registration/transformation_estimation_point_to_plane_weighted.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_svd_scale.h>

// external libs includes
#include <boost/smart_ptr/shared_ptr.hpp>

// project includes
#include <dynamic_robot_localization/common/common.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

enum TransformationEstimationApproach {
	TransformationEstimation2D,
	TransformationEstimationDualQuaternion,
	TransformationEstimationLM,
	TransformationEstimationPointToPlane,
	TransformationEstimationPointToPlaneLLS,
	TransformationEstimationPointToPlaneLLSWeighted,
	TransformationEstimationPointToPlaneWeighted,
	TransformationEstimationSVD,
	TransformationEstimationSVDScale
};


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <macros>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#define GenerateTransformationEstimationTimed(BaseClass, Suffix, TemplatesDeclaration, TemplatesUsage) \
template < DRL_UNPACK_ARGS TemplatesDeclaration > \
class BaseClass##Suffix : public pcl::registration::BaseClass< DRL_UNPACK_ARGS TemplatesUsage > { \
	public: \
		typedef boost::shared_ptr< BaseClass##Suffix< DRL_UNPACK_ARGS TemplatesUsage > > Ptr; \
		typedef boost::shared_ptr< const BaseClass##Suffix< DRL_UNPACK_ARGS TemplatesUsage > > ConstPtr; \
\
		BaseClass##Suffix() : transformation_estimation_elapsed_time_(0) {} \
		virtual ~BaseClass##Suffix() {} \
\
		virtual void estimateRigidTransformation (const pcl::PointCloud<PointSource> &cloud_src, const pcl::PointCloud<PointTarget> &cloud_tgt, const pcl::Correspondences &correspondences, typename pcl::registration::TransformationEstimation< DRL_UNPACK_ARGS TemplatesUsage >::Matrix4 &transformation_matrix) const {\
			pcl::StopWatch timer_; \
			timer_.reset(); \
			pcl::registration::BaseClass< DRL_UNPACK_ARGS TemplatesUsage >::estimateRigidTransformation(cloud_src, cloud_tgt, correspondences, transformation_matrix); \
			transformation_estimation_elapsed_time_ += timer_.getTime(); \
		}\
\
		double getTransformationEstimationElapsedTime() { return transformation_estimation_elapsed_time_; } \
		void resetTransformationEstimationElapsedTime() { transformation_estimation_elapsed_time_ = 0; } \
\
	protected: \
		mutable double transformation_estimation_elapsed_time_; \
};

GenerateTransformationEstimationTimed(TransformationEstimation2D, Timed, (typename PointSource, typename PointTarget, typename Scalar = float), (PointSource, PointTarget, Scalar))
GenerateTransformationEstimationTimed(TransformationEstimationDualQuaternion, Timed, (typename PointSource, typename PointTarget, typename Scalar = float), (PointSource, PointTarget, Scalar))
GenerateTransformationEstimationTimed(TransformationEstimationLM, Timed, (typename PointSource, typename PointTarget, typename Scalar = float), (PointSource, PointTarget, Scalar))
GenerateTransformationEstimationTimed(TransformationEstimationPointToPlane, Timed, (typename PointSource, typename PointTarget, typename Scalar = float), (PointSource, PointTarget, Scalar))
GenerateTransformationEstimationTimed(TransformationEstimationPointToPlaneLLS, Timed, (typename PointSource, typename PointTarget, typename Scalar = float), (PointSource, PointTarget, Scalar))
GenerateTransformationEstimationTimed(TransformationEstimationPointToPlaneLLSWeighted, Timed, (typename PointSource, typename PointTarget, typename Scalar = float), (PointSource, PointTarget, Scalar))
GenerateTransformationEstimationTimed(TransformationEstimationPointToPlaneWeighted, Timed, (typename PointSource, typename PointTarget, typename Scalar = float), (PointSource, PointTarget, Scalar))
GenerateTransformationEstimationTimed(TransformationEstimationSVD, Timed, (typename PointSource, typename PointTarget, typename Scalar = float), (PointSource, PointTarget, Scalar))
GenerateTransformationEstimationTimed(TransformationEstimationSVDScale, Timed, (typename PointSource, typename PointTarget, typename Scalar = float), (PointSource, PointTarget, Scalar))
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </macros>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


} /* namespace dynamic_robot_localization */

