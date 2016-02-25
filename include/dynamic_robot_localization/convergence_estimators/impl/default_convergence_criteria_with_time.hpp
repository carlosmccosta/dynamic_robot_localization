/**\file default_convergence_criteria_with_time.hpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/convergence_estimators/default_convergence_criteria_with_time.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <DefaultConvergenceCriteriaWithTime-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename Scalar>
bool DefaultConvergenceCriteriaWithTime<Scalar>::hasConverged() {
	double elapsed_time = convergence_timer_.getTimeSeconds();
	convergence_state_time_limit_reached_ = false;
	if (convergence_time_limit_seconds_ >= 0.0 && elapsed_time > convergence_time_limit_seconds_) {
		ROS_WARN_STREAM("[DefaultConvergenceCriteriaWithTime::hasConverged] Convergence time limit of " << convergence_time_limit_seconds_ << " seconds exceeded (elapsed time: " << elapsed_time << ")" \
				<< " | Iteration: " << pcl::registration::DefaultConvergenceCriteria<Scalar>::iterations_ \
				<< " | CorrespondencesCurrentMSE: " << pcl::registration::DefaultConvergenceCriteria<Scalar>::correspondences_cur_mse_);

		pcl::registration::DefaultConvergenceCriteria<Scalar>::convergence_state_ = pcl::registration::DefaultConvergenceCriteria<Scalar>::CONVERGENCE_CRITERIA_ITERATIONS;
		convergence_state_time_limit_reached_ = true;
		return true;
	} else {
		bool converged = pcl::registration::DefaultConvergenceCriteria<Scalar>::hasConverged();
		ROS_DEBUG_STREAM("[DefaultConvergenceCriteriaWithTime::hasConverged]:" \
				<< "\n\t Convergence state: " << getConvergenceStateString() \
				<< "\n\t Current convergence time: " << elapsed_time \
				<< "\n\t Convergence time limit: " << convergence_time_limit_seconds_ \
				<< "\n\t Iteration: " << pcl::registration::DefaultConvergenceCriteria<Scalar>::iterations_ \
				<< "\n\t CorrespondencesCurrentMeanSquareError: " << pcl::registration::DefaultConvergenceCriteria<Scalar>::correspondences_cur_mse_ \
				<< "\n\t Current convergence transformation is:" \
				<< "\n\t\t " << std::setw(18) << transformation_(0, 0) << " " << std::setw(18) <<  transformation_(0, 1) << " " << std::setw(18) <<  transformation_(0, 2) << " " << std::setw(18) <<  transformation_(0, 3) \
				<< "\n\t\t " << std::setw(18) << transformation_(1, 0) << " " << std::setw(18) <<  transformation_(1, 1) << " " << std::setw(18) <<  transformation_(1, 2) << " " << std::setw(18) <<  transformation_(1, 3) \
				<< "\n\t\t " << std::setw(18) << transformation_(2, 0) << " " << std::setw(18) <<  transformation_(2, 1) << " " << std::setw(18) <<  transformation_(2, 2) << " " << std::setw(18) <<  transformation_(2, 3) \
				<< "\n\t\t " << std::setw(18) << transformation_(3, 0) << " " << std::setw(18) <<  transformation_(3, 1) << " " << std::setw(18) <<  transformation_(3, 2) << " " << std::setw(18) <<  transformation_(3, 3) << "\n");

		if (!pcl_isfinite(transformation_(0, 0)) || !pcl_isfinite(transformation_(0, 1)) || !pcl_isfinite(transformation_(0, 2)) || !pcl_isfinite(transformation_(0, 3)) ||
			!pcl_isfinite(transformation_(1, 0)) || !pcl_isfinite(transformation_(1, 1)) || !pcl_isfinite(transformation_(1, 2)) || !pcl_isfinite(transformation_(1, 3)) ||
			!pcl_isfinite(transformation_(2, 0)) || !pcl_isfinite(transformation_(2, 1)) || !pcl_isfinite(transformation_(2, 2)) || !pcl_isfinite(transformation_(2, 3)) ||
			!pcl_isfinite(transformation_(3, 0)) || !pcl_isfinite(transformation_(3, 1)) || !pcl_isfinite(transformation_(3, 2)) || !pcl_isfinite(transformation_(3, 3))) {
			ROS_WARN("[DefaultConvergenceCriteriaWithTime::hasConverged] Rejected estimated transformation with NaN values!");
			return true; // a transform with NaNs will cause a crash because of kd-tree search
		}

		return converged;
	}
}


template<typename Scalar>
void DefaultConvergenceCriteriaWithTime<Scalar>::resetConvergenceTimer() {
	convergence_timer_.reset();
}


template<typename Scalar>
std::string DefaultConvergenceCriteriaWithTime<Scalar>::getConvergenceStateString() {
	if (convergence_state_time_limit_reached_) { return "CONVERGENCE_CRITERIA_TIME_LIMIT"; }
	typename pcl::registration::DefaultConvergenceCriteria<Scalar>::ConvergenceState convergence_state = pcl::registration::DefaultConvergenceCriteria<Scalar>::getConvergenceState();
	switch (convergence_state) {
		case pcl::registration::DefaultConvergenceCriteria<Scalar>::CONVERGENCE_CRITERIA_NOT_CONVERGED: 		{ return "CONVERGENCE_CRITERIA_NOT_CONVERGED"; 		break; }
		case pcl::registration::DefaultConvergenceCriteria<Scalar>::CONVERGENCE_CRITERIA_ITERATIONS: 			{ return "CONVERGENCE_CRITERIA_ITERATIONS"; 		break; }
		case pcl::registration::DefaultConvergenceCriteria<Scalar>::CONVERGENCE_CRITERIA_TRANSFORM: 			{ return "CONVERGENCE_CRITERIA_TRANSFORM"; 			break; }
		case pcl::registration::DefaultConvergenceCriteria<Scalar>::CONVERGENCE_CRITERIA_ABS_MSE: 				{ return "CONVERGENCE_CRITERIA_ABS_MSE"; 			break; }
		case pcl::registration::DefaultConvergenceCriteria<Scalar>::CONVERGENCE_CRITERIA_REL_MSE: 				{ return "CONVERGENCE_CRITERIA_REL_MSE"; 			break; }
		case pcl::registration::DefaultConvergenceCriteria<Scalar>::CONVERGENCE_CRITERIA_NO_CORRESPONDENCES: 	{ return "CONVERGENCE_CRITERIA_NO_CORRESPONDENCES"; break; }
		default: break;
	}
	return "";
}

template<typename Scalar>
double DefaultConvergenceCriteriaWithTime<Scalar>::getRootMeanSquareErrorOfRegistrationCorrespondences() {
	double mse = pcl::registration::DefaultConvergenceCriteria<Scalar>::correspondences_cur_mse_;
	if (mse >= 0.0) {
		return std::sqrt(mse);
	} else {
		return std::numeric_limits<double>::max();
	}
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </DefaultConvergenceCriteriaWithTime-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// =============================================================================   <private-section>   =========================================================================
// =============================================================================   </private-section>  =========================================================================

} /* namespace dynamic_robot_localization */
