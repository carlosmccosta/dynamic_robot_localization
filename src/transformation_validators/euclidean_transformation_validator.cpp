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
		max_transformation_distance_(2.0),
		max_alignment_fitness_(0.1),
		max_outliers_percentage_(0.6) {}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <EuclideanTransformationValidator-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void EuclideanTransformationValidator::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle) {
	private_node_handle->param("max_transformation_angle", max_transformation_angle_, 1.59);
	private_node_handle->param("max_transformation_distance", max_transformation_distance_, 2.0);
	private_node_handle->param("max_alignment_fitness", max_alignment_fitness_, 0.1);
	private_node_handle->param("max_outliers_percentage", max_outliers_percentage_, 0.6);
}

bool EuclideanTransformationValidator::validateNewLocalizationPose(const tf2::Transform& initial_guess, tf2::Transform& new_pose, double alignment_fitness, double outliers_percentage) {
	double transform_distance = std::abs(new_pose.getOrigin().length() - initial_guess.getOrigin().length());
	double transform_angle = std::abs(new_pose.getRotation().getAngle() - initial_guess.getRotation().getAngle());

	if (alignment_fitness < max_alignment_fitness_
			&& outliers_percentage < max_outliers_percentage_
	        && transform_distance < max_transformation_distance_
	        && transform_angle < max_transformation_angle_) {

		ROS_DEBUG_STREAM("EuclideanTransformationValidator accepted new pose-> " \
					<< "\n\t translation: " 		<< transform_distance \
					<< "\n\t rotation: " 			<< transform_angle \
					<< "\n\t alignment_fitness: " 	<< alignment_fitness \
					<< "\n\t outliers_percentage: "	<< outliers_percentage);
		return true;
	}

	ROS_DEBUG_STREAM("EuclideanTransformationValidator rejected new pose -> " \
			<< "\n\t translation: " 		<< transform_distance \
			<< "\n\t rotation: " 			<< transform_angle \
			<< "\n\t alignment_fitness: " 	<< alignment_fitness \
			<< "\n\t outliers_percentage: "	<< outliers_percentage);
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
