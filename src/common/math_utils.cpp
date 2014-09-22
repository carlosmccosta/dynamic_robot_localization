/**\file math_utils.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/common/math_utils.h>
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
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </math_utils-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


} /* namespace math_utils */
} /* namespace dynamic_robot_localization */


