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
	if (convergence_time_limit_seconds_ >= 0.0 && elapsed_time > convergence_time_limit_seconds_) {
		ROS_WARN_STREAM("[DefaultConvergenceCriteriaWithTime::hasConverged] Convergence time limit of (" << convergence_time_limit_seconds_ << ") seconds exceeded (" << elapsed_time << ")" \
				<< " | Iteration: " << pcl::registration::DefaultConvergenceCriteria<Scalar>::iterations_ \
				<< " | CorrespondencesCurrentMeanSquareError: " << pcl::registration::DefaultConvergenceCriteria<Scalar>::correspondences_cur_mse_);

		pcl::registration::DefaultConvergenceCriteria<Scalar>::convergence_state_ = pcl::registration::DefaultConvergenceCriteria<Scalar>::CONVERGENCE_CRITERIA_ITERATIONS;
		return true;
	} else {
		bool converged = pcl::registration::DefaultConvergenceCriteria<Scalar>::hasConverged();
		ROS_DEBUG_STREAM("[DefaultConvergenceCriteriaWithTime::hasConverged] Convergence time within limits (" << elapsed_time << "\t < " << convergence_time_limit_seconds_ << ")" \
				<< " | Iteration: " << pcl::registration::DefaultConvergenceCriteria<Scalar>::iterations_ \
				<< " | CorrespondencesCurrentMeanSquareError: " << pcl::registration::DefaultConvergenceCriteria<Scalar>::correspondences_cur_mse_);

		ROS_DEBUG("[DefaultConvergenceCriteriaWithTime::hasConverged] Current convergence transformation is:\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n",
				transformation_(0, 0), transformation_(0, 1), transformation_(0, 2), transformation_(0, 3),
				transformation_(1, 0), transformation_(1, 1), transformation_(1, 2), transformation_(1, 3),
				transformation_(2, 0), transformation_(2, 1), transformation_(2, 2), transformation_(2, 3),
				transformation_(3, 0), transformation_(3, 1), transformation_(3, 2), transformation_(3, 3));

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
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </DefaultConvergenceCriteriaWithTime-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// =============================================================================   <private-section>   =========================================================================
// =============================================================================   </private-section>  =========================================================================

} /* namespace dynamic_robot_localization */
