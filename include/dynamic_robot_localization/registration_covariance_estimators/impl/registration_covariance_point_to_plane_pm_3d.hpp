/**\file registration_covariance_point_to_plane_pm_3d.hpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/registration_covariance_estimators/registration_covariance_point_to_plane_pm_3d.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <RegistrationCovariancePointToPlanePM3D-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
bool RegistrationCovariancePointToPlanePM3D<PointT>::computeRegistrationCovariance(const pcl::PointCloud<PointT>& reference_cloud_correspondences, const pcl::PointCloud<PointT>& ambient_cloud_orrespondences,
		const Eigen::Matrix4f& registration_corrections, Eigen::MatrixXd& covariance_out, double sensor_std_dev_noise) {
	if (reference_cloud_correspondences.empty() || ambient_cloud_orrespondences.empty()) {
		return false;
	}

	size_t number_points = std::min(reference_cloud_correspondences.size(), ambient_cloud_orrespondences.size());

	covariance_out = Eigen::MatrixXd(Eigen::MatrixXd::Zero(6,6));
	Eigen::MatrixXd J_hessian(Eigen::MatrixXd::Zero(6,6));
	Eigen::MatrixXd d2J_dReadingdX(Eigen::MatrixXd::Zero(6, number_points));
	Eigen::MatrixXd d2J_dReferencedX(Eigen::MatrixXd::Zero(6, number_points));

	double correction_roll, correction_pitch, correction_yaw;
	math_utils::getRollPitchYawFromMatrix(registration_corrections, correction_roll, correction_pitch, correction_yaw);

	double correction_x = registration_corrections(0,3);
	double correction_y = registration_corrections(1,3);
	double correction_z = registration_corrections(2,3);

	Eigen::Matrix<double, Eigen::Dynamic, 1> tmp_vector_6(Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(6));

	int valid_points_count = 0;
	double reference_point_normal_x = 1.0;
	double reference_point_normal_y = 1.0;
	double reference_point_normal_z = 1.0;

	for(size_t i = 0; i < number_points; ++i) {
		const PointT& ambient_point = ambient_cloud_orrespondences[i];
		const PointT& reference_point = reference_cloud_correspondences[i];

		if (use_normals_) {
			reference_point_normal_x = reference_point.normal_x;
			reference_point_normal_y = reference_point.normal_y;
			reference_point_normal_z = reference_point.normal_z;
		}

		double reading_range = std::sqrt(ambient_point.x * ambient_point.x + ambient_point.y * ambient_point.y + ambient_point.z * ambient_point.z);
		double ambient_point_direction_x = ambient_point.x / reading_range;
		double ambient_point_direction_y = ambient_point.y / reading_range;
		double ambient_point_direction_z = ambient_point.z / reading_range;

		double reference_range = std::sqrt(reference_point.x * reference_point.x + reference_point.y * reference_point.y + reference_point.z * reference_point.z);
		double reference_point_direction_x = reference_point.x / reference_range;
		double reference_point_direction_y = reference_point.y / reference_range;
		double reference_point_direction_z = reference_point.z / reference_range;

		double n_correction_roll  = reference_point_normal_z*ambient_point_direction_y - reference_point_normal_y*ambient_point_direction_z;
		double n_correction_pitch = reference_point_normal_x*ambient_point_direction_z - reference_point_normal_z*ambient_point_direction_x;
		double n_correction_yaw   = reference_point_normal_y*ambient_point_direction_x - reference_point_normal_x*ambient_point_direction_y;

		double E = 	reference_point_normal_x*(ambient_point.x - correction_yaw*ambient_point.y + correction_pitch*ambient_point.z 	+ correction_x - reference_point.x);
		E += 		reference_point_normal_y*(correction_yaw*ambient_point.x + ambient_point.y - correction_roll *ambient_point.z 	+ correction_y - reference_point.y);
		E += 		reference_point_normal_z*(-correction_pitch*ambient_point.x + correction_roll*ambient_point.y + ambient_point.z + correction_z - reference_point.z);

		double N_reading = reference_point_normal_x*(ambient_point_direction_x - correction_yaw*ambient_point_direction_y + correction_pitch*ambient_point_direction_z);
		N_reading 		+= reference_point_normal_y*(correction_yaw*ambient_point_direction_x + ambient_point_direction_y - correction_roll*ambient_point_direction_z);
		N_reading 		+= reference_point_normal_z*(-correction_pitch*ambient_point_direction_x + correction_roll*ambient_point_direction_y + ambient_point_direction_z);

		double N_reference = -(reference_point_normal_x*reference_point_direction_x + reference_point_normal_y*reference_point_direction_y + reference_point_normal_z*reference_point_direction_z);

		// update the hessian and d2J/dzdx
		tmp_vector_6 << reference_point_normal_x, reference_point_normal_y, reference_point_normal_z, reading_range * n_correction_roll, reading_range * n_correction_pitch, reading_range * n_correction_yaw;
		J_hessian += tmp_vector_6 * tmp_vector_6.transpose();
		tmp_vector_6 << reference_point_normal_x * N_reading, reference_point_normal_y * N_reading, reference_point_normal_z * N_reading, n_correction_roll * (E + reading_range * N_reading), n_correction_pitch * (E + reading_range * N_reading), n_correction_yaw * (E + reading_range * N_reading);
		d2J_dReadingdX.block(0,valid_points_count,6,1) = tmp_vector_6;
		tmp_vector_6 << reference_point_normal_x * N_reference, reference_point_normal_y * N_reference, reference_point_normal_z * N_reference, reference_range * n_correction_roll * N_reference, reference_range * n_correction_pitch * N_reference, reference_range * n_correction_yaw * N_reference;
		d2J_dReferencedX.block(0,valid_points_count,6,1) = tmp_vector_6;

		++valid_points_count;
	}

	Eigen::MatrixXd d2J_dZdX(Eigen::MatrixXd::Zero(6, 2 * valid_points_count));
	d2J_dZdX.block(0,0,6,valid_points_count) = d2J_dReadingdX.block(0,0,6,valid_points_count);
	d2J_dZdX.block(0,valid_points_count,6,valid_points_count) = d2J_dReferencedX.block(0,0,6,valid_points_count);

//	Eigen::MatrixXd inv_J_hessian = J_hessian.inverse();
	Eigen::FullPivLU<Eigen::MatrixXd> lu(J_hessian);
	Eigen::MatrixXd inv_J_hessian = lu.inverse();

	covariance_out = d2J_dZdX * d2J_dZdX.transpose();
	covariance_out = inv_J_hessian * covariance_out * inv_J_hessian;
	covariance_out = (sensor_std_dev_noise * sensor_std_dev_noise) * covariance_out;

	return true;
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </RegistrationCovariancePointToPlanePM3D-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <template instantiations>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </template instantiations>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

} /* namespace dynamic_robot_localization */
