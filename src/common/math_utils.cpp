/**\file math_utils.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/common/common.h>
#include <dynamic_robot_localization/common/impl/math_utils.hpp>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {
namespace math_utils {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <math_utils-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
PointPosition computePointPositionInRelationToLinePoints(
		float line_start_point_x, float line_start_point_y,
		float line_end_point_x, float line_end_point_y,
		float query_point_x, float query_point_y) {
	float cross_product_z = (line_end_point_x - line_start_point_x) * (query_point_y - line_start_point_y) - (line_end_point_y - line_start_point_y) * (query_point_x - line_start_point_x);
	return computePointPosition(cross_product_z);
}


PointPosition computePointPositionInRelationToLine(
		float point_on_line_x, float point_on_line_y,
		float line_orientation_x, float line_orientation_y,
		float query_point_x, float query_point_y) {
	float cross_product_z = line_orientation_x * (query_point_y - point_on_line_y) - line_orientation_y * (query_point_x - point_on_line_x);
	return computePointPosition(cross_product_z);
}


PointPosition computePointPosition(float cross_product_z) {
	if (cross_product_z > 0.0) {
		return LEFT_LINE_SIDE;
	} else if (cross_product_z < 0.0) {
		return RIGHT_LINE_SIDE;
	} else {
		return ON_TOP_OF_LINE;
	}
}


bool isTransformValid(const tf2::Transform& transform) {
	tf2::Vector3 position = transform.getOrigin();
	tf2::Quaternion orientation = transform.getRotation();
	if (!pcl_isfinite(position.x()) || !pcl_isfinite(position.y()) || !pcl_isfinite(position.z())
			|| !pcl_isfinite(orientation.x()) || !pcl_isfinite(orientation.y()) || !pcl_isfinite(orientation.z()) || !pcl_isfinite(orientation.w())) {
		return false;
	}

	return true;
}


void getRollPitchYawFromMatrix(const Eigen::Matrix4f& matrix, double& roll_out, double& pitch_out, double& yaw_out) {
	roll_out = atan2(matrix(2, 1), matrix(2, 2));
	pitch_out = asin(-matrix(2, 0));
//	pitch_out = atan2(-matrix(2,0), std::sqrt(matrix(2,1) * matrix(2,1) + matrix(2,2) * matrix(2,2)));
	yaw_out = atan2(matrix(1, 0), matrix(0, 0));
}


void getRollPitchYawFromMatrixUsigTF2(const Eigen::Matrix4f& matrix, double& roll_out, double& pitch_out, double& yaw_out) {
	tf2::Transform transform;
	Eigen::Matrix4d doubleMatrix(matrix.cast<double>());
	transform.setFromOpenGLMatrix(doubleMatrix.data());
	transform.getBasis().getRPY(roll_out, pitch_out, yaw_out);
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </math_utils-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


} /* namespace math_utils */
} /* namespace dynamic_robot_localization */



// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <template instantiations>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#ifndef DRL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>

#define PCL_INSTANTIATE_DRLMathUtilsIsTransformValid(T) template bool dynamic_robot_localization::math_utils::isTransformValid<T>(const Eigen::Matrix<T, 4, 4>&);
PCL_INSTANTIATE(DRLMathUtilsIsTransformValid, DRL_SCALAR_TYPES)

#define PCL_INSTANTIATE_DRLMathUtilsConvertTransformToString(T) template std::string dynamic_robot_localization::math_utils::convertTransformToString<T>(const Eigen::Matrix<T, 4, 4>&, const std::string&, const std::string&,  const std::string&);
PCL_INSTANTIATE(DRLMathUtilsConvertTransformToString, DRL_SCALAR_TYPES)
#endif
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </template instantiations>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
