/**\file euclidean_transformation_validator.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/transformation_validators/euclidean_transformation_validator.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
EuclideanTransformationValidator::EuclideanTransformationValidator() :
		max_transformation_angle_(1.59),
		max_transformation_distance_(0.1),
		max_new_pose_diff_angle_(1.59),
		max_new_pose_diff_distance_(0.2),
		max_root_mean_square_error_(0.05),
		max_outliers_percentage_(0.6),
		min_inliers_angular_distribution_(0.125),
		max_outliers_angular_distribution_(0.875) {}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <EuclideanTransformationValidator-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void EuclideanTransformationValidator::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace) {
	private_node_handle->param(configuration_namespace + "max_transformation_angle", max_transformation_angle_, 0.7);
	private_node_handle->param(configuration_namespace + "max_transformation_distance", max_transformation_distance_, 0.1);
	private_node_handle->param(configuration_namespace + "max_new_pose_diff_angle", max_new_pose_diff_angle_, 1.59);
	private_node_handle->param(configuration_namespace + "max_new_pose_diff_distance", max_new_pose_diff_distance_, 0.2);
	private_node_handle->param(configuration_namespace + "max_root_mean_square_error", max_root_mean_square_error_, 0.05);
	private_node_handle->param(configuration_namespace + "min_overriding_root_mean_square_error", min_overriding_root_mean_square_error_, 0.01);
	private_node_handle->param(configuration_namespace + "max_outliers_percentage", max_outliers_percentage_, 0.6);
	private_node_handle->param(configuration_namespace + "min_overriding_outliers_percentage", min_overriding_outliers_percentage_, 0.1);
	private_node_handle->param(configuration_namespace + "min_inliers_angular_distribution", min_inliers_angular_distribution_, 0.125);
	private_node_handle->param(configuration_namespace + "max_outliers_angular_distribution", max_outliers_angular_distribution_, 0.875);
}

bool EuclideanTransformationValidator::validateNewLocalizationPose(const tf2::Transform& last_accepted_pose, const tf2::Transform& initial_guess, tf2::Transform& new_pose,
		double root_mean_square_error, double outliers_percentage, double inliers_angular_distribution, double outliers_angular_distribution) {
	double transform_distance = (new_pose.getOrigin() - initial_guess.getOrigin()).length();
	double transform_angle = std::abs(new_pose.getRotation().normalize().angleShortestPath(initial_guess.getRotation().normalize()));

	double new_pose_distance = (new_pose.getOrigin() - last_accepted_pose.getOrigin()).length();
	double new_pose_angle = std::abs(new_pose.getRotation().normalize().angleShortestPath(last_accepted_pose.getRotation().normalize()));

	std::stringstream validation_info;
	validation_info << "\n\t correction translation: " 			<< transform_distance \
					<< "\n\t correction rotation: " 			<< transform_angle \
					<< "\n\t new pose diff translation: " 		<< new_pose_distance \
					<< "\n\t new pose diff rotation: " 			<< new_pose_angle \
					<< "\n\t root_mean_square_error: " 			<< root_mean_square_error \
					<< "\n\t outliers_percentage: "				<< outliers_percentage \
					<< "\n\t inliers_angular_distribution: "	<< inliers_angular_distribution \
					<< "\n\t outliers_angular_distribution: "	<< outliers_angular_distribution;

	if ((root_mean_square_error < min_overriding_root_mean_square_error_ &&
			outliers_percentage < min_overriding_outliers_percentage_) ||
			(root_mean_square_error < max_root_mean_square_error_
			&& outliers_percentage < max_outliers_percentage_
			&& (max_transformation_distance_ < 0 || transform_distance < max_transformation_distance_)
			&& (max_transformation_angle_ < 0 || transform_angle < max_transformation_angle_)
			&& (max_new_pose_diff_distance_ < 0 || new_pose_distance < max_new_pose_diff_distance_)
			&& (max_new_pose_diff_angle_ < 0 || new_pose_angle < max_new_pose_diff_angle_)
			&& inliers_angular_distribution > min_inliers_angular_distribution_
			&& outliers_angular_distribution < max_outliers_angular_distribution_)) {

		ROS_DEBUG_STREAM("EuclideanTransformationValidator accepted new pose at time " << ros::Time::now() << " -> " << validation_info.str());
		return true;
	}

	ROS_WARN_STREAM("EuclideanTransformationValidator rejected new pose at time " << ros::Time::now() << " -> " << validation_info.str());
	return false;
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </EuclideanTransformationValidator-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// =============================================================================   <private-section>   =========================================================================
// =============================================================================   </private-section>  =========================================================================

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <template instantiations>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </template instantiations>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

} /* namespace dynamic_robot_localization */
