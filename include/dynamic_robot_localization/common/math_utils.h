#pragma once

/**\file math_utils.h
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
#include <utility>
#include <iomanip>

// ROS includes
#include <tf2/LinearMath/Transform.h>

// PCL includes
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/transformation_estimation_3point.h>

// external libs includes
#include <Eigen/Core>

// project includes
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

// ###############################################################################   math_utils   ##############################################################################
namespace math_utils {

enum PointPosition {
	LEFT_LINE_SIDE,
	RIGHT_LINE_SIDE,
	ON_TOP_OF_LINE
};


PointPosition computePointPositionInRelationToLinePoints(
		float line_start_point_x, float line_start_point_y,
		float line_end_point_x, float line_end_point_y,
		float query_point_x, float query_point_y);

PointPosition computePointPositionInRelationToLine(
		float point_on_line_x, float point_on_line_y,
		float line_orientation_x, float line_orientation_y,
		float query_point_x, float query_point_y);

PointPosition computePointPosition(float cross_product_z);

void computeTransformationFromMatrices(const Eigen::Matrix4f& source_matrix, const Eigen::Matrix4f& target_matrix, Eigen::Matrix4f& transformation_matrix);


bool isTransformValid(const tf2::Transform& transform);

template <typename Scalar>
bool isTransformValid(const Eigen::Matrix<Scalar, 4, 4>& transform);

template <typename Scalar>
std::string convertTransformToString(const Eigen::Matrix<Scalar, 4, 4>& transform, const std::string& line_prefix = "\n\t\t[ ", const std::string& line_suffix = " ]",  const std::string& number_separator = " | ");

template <typename PairFirst, typename PairSecond>
bool sortFunctionForPairSecondValueAscendingOrder(const std::pair<PairFirst, PairSecond> &left, const std::pair<PairFirst, PairSecond> &right) { return left.second < right.second; }

template <typename PairFirst, typename PairSecond>
bool sortFunctionForPairSecondValueDescendingOrder(const std::pair<PairFirst, PairSecond> &left, const std::pair<PairFirst, PairSecond> &right) { return left.second > right.second; }

void getRollPitchYawFromMatrix(const Eigen::Matrix4f& matrix, double& roll_out, double& pitch_out, double& yaw_out);
void getRollPitchYawFromMatrixUsigTF2(const Eigen::Matrix4f& matrix, double& roll_out, double& pitch_out, double& yaw_out);


} /* namespace math_utils */
} /* namespace dynamic_robot_localization */


#ifdef DRL_NO_PRECOMPILE
#include <dynamic_robot_localization/common/impl/math_utils.hpp>
#endif
