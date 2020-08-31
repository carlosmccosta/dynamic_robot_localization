/**\file math_utils.hpp
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


template <typename Scalar>
bool isTransformValid(const Eigen::Matrix<Scalar, 4, 4>& transform) {
	if (!std::isfinite(transform(0, 0)) || !std::isfinite(transform(0, 1)) || !std::isfinite(transform(0, 2)) || !std::isfinite(transform(0, 3)) ||
		!std::isfinite(transform(1, 0)) || !std::isfinite(transform(1, 1)) || !std::isfinite(transform(1, 2)) || !std::isfinite(transform(1, 3)) ||
		!std::isfinite(transform(2, 0)) || !std::isfinite(transform(2, 1)) || !std::isfinite(transform(2, 2)) || !std::isfinite(transform(2, 3)) ||
		!std::isfinite(transform(3, 0)) || !std::isfinite(transform(3, 1)) || !std::isfinite(transform(3, 2)) || !std::isfinite(transform(3, 3))) {
		return false;
	}
	return true;
}


template <typename Scalar>
std::string convertTransformToString(const Eigen::Matrix<Scalar, 4, 4>& transform, const std::string& line_prefix, const std::string& line_suffix, const std::string& number_separator) {
	std::stringstream ss;
	ss	<< line_prefix << std::setw(18) << transform(0, 0) << number_separator << std::setw(18) <<  transform(0, 1) << number_separator << std::setw(18) <<  transform(0, 2) << number_separator << std::setw(18) <<  transform(0, 3) << line_suffix
		<< line_prefix << std::setw(18) << transform(1, 0) << number_separator << std::setw(18) <<  transform(1, 1) << number_separator << std::setw(18) <<  transform(1, 2) << number_separator << std::setw(18) <<  transform(1, 3) << line_suffix
		<< line_prefix << std::setw(18) << transform(2, 0) << number_separator << std::setw(18) <<  transform(2, 1) << number_separator << std::setw(18) <<  transform(2, 2) << number_separator << std::setw(18) <<  transform(2, 3) << line_suffix
		<< line_prefix << std::setw(18) << transform(3, 0) << number_separator << std::setw(18) <<  transform(3, 1) << number_separator << std::setw(18) <<  transform(3, 2) << number_separator << std::setw(18) <<  transform(3, 3) << line_suffix;
	return ss.str();
}


} /* namespace math_utils */
} /* namespace dynamic_robot_localization */
