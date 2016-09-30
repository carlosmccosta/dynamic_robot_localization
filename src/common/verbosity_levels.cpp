/**\file verbosity_levels.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/common/verbosity_levels.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {
namespace verbosity_levels {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <verbosity_levels-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	bool setVerbosityLevelPCL(std::string level) {
		if (level == "VERBOSE") {
			pcl::console::setVerbosityLevel(pcl::console::L_VERBOSE);
		} else if (level == "DEBUG") {
			pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);
		} else if (level == "INFO") {
			pcl::console::setVerbosityLevel(pcl::console::L_INFO);
		} else if (level == "WARN") {
			pcl::console::setVerbosityLevel(pcl::console::L_WARN);
		} else if (level == "ERROR") {
			pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
		} else if (level == "ALWAYS") {
			pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
		} else {
			return false;
		}

		return true;
	}

	bool setVerbosityLevelROS(std::string level) {
		if (level == "DEBUG") {
			if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
				ros::console::notifyLoggerLevelsChanged();
			}
		} else if (level == "INFO") {
			if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) {
				ros::console::notifyLoggerLevelsChanged();
			}
		} else if (level == "WARN") {
			if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn)) {
				ros::console::notifyLoggerLevelsChanged();
			}
		} else if (level == "ERROR") {
			if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Error)) {
				ros::console::notifyLoggerLevelsChanged();
			}
		} else if (level == "FATAL") {
			if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Fatal)) {
				ros::console::notifyLoggerLevelsChanged();
			}
		} else {
			return false;
		}

		return true;
	}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </verbosity_levels-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


} /* namespace verbosity_levels */
} /* namespace dynamic_robot_localization */
