/**\file registration_covariance_point_to_point_3d.hpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/registration_covariance_estimators/registration_covariance_point_to_point_3d.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <RegistrationCovariancePointToPoint3D-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
bool RegistrationCovariancePointToPoint3D<PointT>::computeRegistrationCovariance(const pcl::PointCloud<PointT>& reference_cloud_correspondences, const pcl::PointCloud<PointT>& ambient_cloud_orrespondences,
		const Eigen::Matrix4f& registration_corrections, Eigen::MatrixXd& covariance_out, double sensor_std_dev_noise) {
	if (reference_cloud_correspondences.empty() || ambient_cloud_orrespondences.empty()) {
		return false;
	}

	double correction_x = registration_corrections(0, 3);
	double correction_y = registration_corrections(1, 3);
	double correction_z = registration_corrections(2, 3);

	double correction_roll, correction_pitch, correction_yaw;
	math_utils::getRollPitchYawFromMatrix(registration_corrections, correction_roll, correction_pitch, correction_yaw);

	double cos_roll = cos(correction_roll);
	double sin_roll = sin(correction_roll);
	double cos_pitch = cos(correction_pitch);
	double sin_pitch = sin(correction_pitch);
	double cos_yaw = cos(correction_yaw);
	double sin_yaw = sin(correction_yaw);

	Eigen::MatrixXd d2J_dX2(6, 6);
	d2J_dX2 = Eigen::MatrixXd::Zero(6, 6);
	size_t ambient_cloud_orrespondences_size = ambient_cloud_orrespondences.points.size();

	for (size_t s = 0; s < ambient_cloud_orrespondences_size; ++s) {
		double pix = ambient_cloud_orrespondences[s].x;
		double piy = ambient_cloud_orrespondences[s].y;
		double piz = ambient_cloud_orrespondences[s].z;
		double qix = reference_cloud_correspondences[s].x;
		double qiy = reference_cloud_correspondences[s].y;
		double qiz = reference_cloud_correspondences[s].z;

		double 	d2J_dx2 , d2J_dydx, d2J_dzdx, d2J_dadx, d2J_dbdx, d2J_dcdx,
				d2J_dxdy, d2J_dy2 , d2J_dzdy, d2J_dady, d2J_dbdy, d2J_dcdy,
				d2J_dxdz, d2J_dydz, d2J_dz2 , d2J_dadz, d2J_dbdz, d2J_dcdz,
				d2J_dxda, d2J_dyda, d2J_dzda, d2J_da2 , d2J_dbda, d2J_dcda,
				d2J_dxdb, d2J_dydb, d2J_dzdb, d2J_dadb, d2J_db2 , d2J_dcdb,
				d2J_dxdc, d2J_dydc, d2J_dzdc, d2J_dadc, d2J_dbdc, d2J_dc2;

		d2J_dx2 = 2;
		d2J_dy2 = 2;
		d2J_dz2 = 2;
		d2J_dydx = 0;
		d2J_dxdy = 0;
		d2J_dzdx = 0;
		d2J_dxdz = 0;
		d2J_dydz = 0;
		d2J_dzdy = 0;
		d2J_da2 = (piz * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) - piy * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll) + pix * cos_yaw * cos_pitch) * (2 * piz * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) - 2 * piy * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll) + 2 * pix * cos_yaw * cos_pitch) - (2 * piy * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll) - 2 * piz * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + 2 * pix * cos_pitch * sin_yaw) * (correction_y - qiy + piy * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll) - piz * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + pix * cos_pitch * sin_yaw) + (piy * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll) - piz * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + pix * cos_pitch * sin_yaw) * (2 * piy * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll) - 2 * piz * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + 2 * pix * cos_pitch * sin_yaw) - (2 * piz * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) - 2 * piy * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll) + 2 * pix * cos_yaw * cos_pitch) * (correction_x - qix - piy * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll) + piz * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) + pix * cos_yaw * cos_pitch);
		d2J_db2 = (pix * cos_pitch + piz * cos_roll * sin_pitch + piy * sin_pitch * sin_roll) * (2 * pix * cos_pitch + 2 * piz * cos_roll * sin_pitch + 2 * piy * sin_pitch * sin_roll) - (2 * piz * cos_pitch * cos_roll - 2 * pix * sin_pitch + 2 * piy * cos_pitch * sin_roll) * (correction_z - qiz - pix * sin_pitch + piz * cos_pitch * cos_roll + piy * cos_pitch * sin_roll) - (2 * pix * cos_yaw * cos_pitch + 2 * piz * cos_yaw * cos_roll * sin_pitch + 2 * piy * cos_yaw * sin_pitch * sin_roll) * (correction_x - qix - piy * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll) + piz * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) + pix * cos_yaw * cos_pitch) + (piz * cos_yaw * cos_pitch * cos_roll - pix * cos_yaw * sin_pitch + piy * cos_yaw * cos_pitch * sin_roll) * (2 * piz * cos_yaw * cos_pitch * cos_roll - 2 * pix * cos_yaw * sin_pitch + 2 * piy * cos_yaw * cos_pitch * sin_roll) - (2 * pix * cos_pitch * sin_yaw + 2 * piz * cos_roll * sin_yaw * sin_pitch + 2 * piy * sin_yaw * sin_pitch * sin_roll) * (correction_y - qiy + piy * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll) - piz * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + pix * cos_pitch * sin_yaw) + (piz * cos_pitch * cos_roll * sin_yaw - pix * sin_yaw * sin_pitch + piy * cos_pitch * sin_yaw * sin_roll) * (2 * piz * cos_pitch * cos_roll * sin_yaw - 2 * pix * sin_yaw * sin_pitch + 2 * piy * cos_pitch * sin_yaw * sin_roll);
		d2J_dc2 = (piy * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) + piz * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll)) * (2 * piy * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) + 2 * piz * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll)) + (piy * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + piz * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll)) * (2 * piy * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + 2 * piz * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll)) - (2 * piz * cos_pitch * cos_roll + 2 * piy * cos_pitch * sin_roll) * (correction_z - qiz - pix * sin_pitch + piz * cos_pitch * cos_roll + piy * cos_pitch * sin_roll) + (2 * piy * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll) - 2 * piz * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch)) * (correction_x - qix - piy * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll) + piz * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) + pix * cos_yaw * cos_pitch) + (piy * cos_pitch * cos_roll - piz * cos_pitch * sin_roll) * (2 * piy * cos_pitch * cos_roll - 2 * piz * cos_pitch * sin_roll) - (2 * piy * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll) - 2 * piz * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch)) * (correction_y - qiy + piy * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll) - piz * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + pix * cos_pitch * sin_yaw);
		d2J_dxda = 2 * piz * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) - 2 * piy * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll) - 2 * pix * cos_pitch * sin_yaw;
		d2J_dadx = 2 * piz * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) - 2 * piy * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll) - 2 * pix * cos_pitch * sin_yaw;
		d2J_dyda = 2 * piz * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) - 2 * piy * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll) + 2 * pix * cos_yaw * cos_pitch;
		d2J_dady = 2 * piz * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) - 2 * piy * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll) + 2 * pix * cos_yaw * cos_pitch;
		d2J_dzda = 0;
		d2J_dadz = 0;
		d2J_dxdb = 2 * piz * cos_yaw * cos_pitch * cos_roll - 2 * pix * cos_yaw * sin_pitch + 2 * piy * cos_yaw * cos_pitch * sin_roll;
		d2J_dbdx = 2 * piz * cos_yaw * cos_pitch * cos_roll - 2 * pix * cos_yaw * sin_pitch + 2 * piy * cos_yaw * cos_pitch * sin_roll;
		d2J_dydb = 2 * piz * cos_pitch * cos_roll * sin_yaw - 2 * pix * sin_yaw * sin_pitch + 2 * piy * cos_pitch * sin_yaw * sin_roll;
		d2J_dbdy = 2 * piz * cos_pitch * cos_roll * sin_yaw - 2 * pix * sin_yaw * sin_pitch + 2 * piy * cos_pitch * sin_yaw * sin_roll;
		d2J_dzdb = -2 * pix * cos_pitch - 2 * piz * cos_roll * sin_pitch - 2 * piy * sin_pitch * sin_roll;
		d2J_dbdz = -2 * pix * cos_pitch - 2 * piz * cos_roll * sin_pitch - 2 * piy * sin_pitch * sin_roll;
		d2J_dxdc = 2 * piy * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) + 2 * piz * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll);
		d2J_dcdx = 2 * piy * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) + 2 * piz * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll);
		d2J_dydc = -2 * piy * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) - 2 * piz * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll);
		d2J_dcdy = -2 * piy * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) - 2 * piz * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll);
		d2J_dzdc = 2 * piy * cos_pitch * cos_roll - 2 * piz * cos_pitch * sin_roll;
		d2J_dcdz = 2 * piy * cos_pitch * cos_roll - 2 * piz * cos_pitch * sin_roll;
		d2J_dadb = (2 * piz * cos_pitch * cos_roll * sin_yaw - 2 * pix * sin_yaw * sin_pitch + 2 * piy * cos_pitch * sin_yaw * sin_roll) * (piz * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) - piy * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll) + pix * cos_yaw * cos_pitch) - (2 * piz * cos_yaw * cos_pitch * cos_roll - 2 * pix * cos_yaw * sin_pitch + 2 * piy * cos_yaw * cos_pitch * sin_roll) * (piy * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll) - piz * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + pix * cos_pitch * sin_yaw) + (2 * piz * cos_yaw * cos_pitch * cos_roll - 2 * pix * cos_yaw * sin_pitch + 2 * piy * cos_yaw * cos_pitch * sin_roll) * (correction_y - qiy + piy * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll) - piz * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + pix * cos_pitch * sin_yaw) - (2 * piz * cos_pitch * cos_roll * sin_yaw - 2 * pix * sin_yaw * sin_pitch + 2 * piy * cos_pitch * sin_yaw * sin_roll) * (correction_x - qix - piy * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll) + piz * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) + pix * cos_yaw * cos_pitch);
		d2J_dbda = (piz * cos_pitch * cos_roll * sin_yaw - pix * sin_yaw * sin_pitch + piy * cos_pitch * sin_yaw * sin_roll) * (2 * piz * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) - 2 * piy * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll) + 2 * pix * cos_yaw * cos_pitch) - (piz * cos_yaw * cos_pitch * cos_roll - pix * cos_yaw * sin_pitch + piy * cos_yaw * cos_pitch * sin_roll) * (2 * piy * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll) - 2 * piz * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + 2 * pix * cos_pitch * sin_yaw) + (2 * piz * cos_yaw * cos_pitch * cos_roll - 2 * pix * cos_yaw * sin_pitch + 2 * piy * cos_yaw * cos_pitch * sin_roll) * (correction_y - qiy + piy * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll) - piz * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + pix * cos_pitch * sin_yaw) - (2 * piz * cos_pitch * cos_roll * sin_yaw - 2 * pix * sin_yaw * sin_pitch + 2 * piy * cos_pitch * sin_yaw * sin_roll) * (correction_x - qix - piy * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll) + piz * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) + pix * cos_yaw * cos_pitch);
		d2J_dbdc = (2 * piy * cos_yaw * cos_pitch * cos_roll - 2 * piz * cos_yaw * cos_pitch * sin_roll) * (correction_x - qix - piy * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll) + piz * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) + pix * cos_yaw * cos_pitch) + (2 * piy * cos_pitch * cos_roll * sin_yaw - 2 * piz * cos_pitch * sin_yaw * sin_roll) * (correction_y - qiy + piy * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll) - piz * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + pix * cos_pitch * sin_yaw) - (2 * piy * cos_pitch * cos_roll - 2 * piz * cos_pitch * sin_roll) * (pix * cos_pitch + piz * cos_roll * sin_pitch + piy * sin_pitch * sin_roll) - (2 * piy * cos_roll * sin_pitch - 2 * piz * sin_pitch * sin_roll) * (correction_z - qiz - pix * sin_pitch + piz * cos_pitch * cos_roll + piy * cos_pitch * sin_roll) + (2 * piy * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) + 2 * piz * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll)) * (piz * cos_yaw * cos_pitch * cos_roll - pix * cos_yaw * sin_pitch + piy * cos_yaw * cos_pitch * sin_roll) - (2 * piy * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + 2 * piz * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll)) * (piz * cos_pitch * cos_roll * sin_yaw - pix * sin_yaw * sin_pitch + piy * cos_pitch * sin_yaw * sin_roll);
		d2J_dcdb = (2 * piy * cos_yaw * cos_pitch * cos_roll - 2 * piz * cos_yaw * cos_pitch * sin_roll) * (correction_x - qix - piy * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll) + piz * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) + pix * cos_yaw * cos_pitch) + (2 * piy * cos_pitch * cos_roll * sin_yaw - 2 * piz * cos_pitch * sin_yaw * sin_roll) * (correction_y - qiy + piy * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll) - piz * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + pix * cos_pitch * sin_yaw) - (piy * cos_pitch * cos_roll - piz * cos_pitch * sin_roll) * (2 * pix * cos_pitch + 2 * piz * cos_roll * sin_pitch + 2 * piy * sin_pitch * sin_roll) - (2 * piy * cos_roll * sin_pitch - 2 * piz * sin_pitch * sin_roll) * (correction_z - qiz - pix * sin_pitch + piz * cos_pitch * cos_roll + piy * cos_pitch * sin_roll) + (piy * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) + piz * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll)) * (2 * piz * cos_yaw * cos_pitch * cos_roll - 2 * pix * cos_yaw * sin_pitch + 2 * piy * cos_yaw * cos_pitch * sin_roll) - (piy * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + piz * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll)) * (2 * piz * cos_pitch * cos_roll * sin_yaw - 2 * pix * sin_yaw * sin_pitch + 2 * piy * cos_pitch * sin_yaw * sin_roll);
		d2J_dcda = (2 * piy * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + 2 * piz * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll)) * (correction_x - qix - piy * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll) + piz * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) + pix * cos_yaw * cos_pitch) - (piy * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) + piz * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll)) * (2 * piy * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll) - 2 * piz * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + 2 * pix * cos_pitch * sin_yaw) - (piy * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + piz * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll)) * (2 * piz * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) - 2 * piy * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll) + 2 * pix * cos_yaw * cos_pitch) + (2 * piy * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) + 2 * piz * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll)) * (correction_y - qiy + piy * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll) - piz * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + pix * cos_pitch * sin_yaw);
		d2J_dadc = (2 * piy * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + 2 * piz * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll)) * (correction_x - qix - piy * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll) + piz * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) + pix * cos_yaw * cos_pitch) - (2 * piy * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) + 2 * piz * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll)) * (piy * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll) - piz * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + pix * cos_pitch * sin_yaw) - (2 * piy * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + 2 * piz * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll)) * (piz * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) - piy * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll) + pix * cos_yaw * cos_pitch) + (2 * piy * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) + 2 * piz * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll)) * (correction_y - qiy + piy * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll) - piz * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + pix * cos_pitch * sin_yaw);

		Eigen::MatrixXd d2J_dX2_temp(6, 6);
		d2J_dX2_temp << d2J_dx2, d2J_dydx, d2J_dzdx, d2J_dadx, d2J_dbdx, d2J_dzdx, d2J_dxdy, d2J_dy2, d2J_dzdy, d2J_dady, d2J_dbdy, d2J_dcdy, d2J_dxdz, d2J_dydz, d2J_dz2, d2J_dadz, d2J_dbdz, d2J_dcdz, d2J_dxda, d2J_dyda, d2J_dzda, d2J_da2, d2J_dbda, d2J_dcda, d2J_dxdb, d2J_dydb, d2J_dzdb, d2J_dadb, d2J_db2, d2J_dcdb, d2J_dxdc, d2J_dydc, d2J_dzdc, d2J_dadc, d2J_dbdc, d2J_dc2;
		d2J_dX2 = d2J_dX2 + d2J_dX2_temp;
	}

	Eigen::MatrixXd d2J_dZdX(6, 6 * ambient_cloud_orrespondences_size);
	for (size_t k = 0; k < ambient_cloud_orrespondences_size; ++k) {
		double pix = ambient_cloud_orrespondences.points[k].x;
		double piy = ambient_cloud_orrespondences.points[k].y;
		double piz = ambient_cloud_orrespondences.points[k].z;
		double qix = reference_cloud_correspondences.points[k].x;
		double qiy = reference_cloud_correspondences.points[k].y;
		double qiz = reference_cloud_correspondences.points[k].z;

		Eigen::MatrixXd d2J_dZdX_temp(6, 6);
		double 	d2J_dpix_dx, d2J_dpiy_dx, d2J_dpiz_dx, d2J_dqix_dx, d2J_dqiy_dx, d2J_dqiz_dx,
				d2J_dpix_dy, d2J_dpiy_dy, d2J_dpiz_dy, d2J_dqix_dy, d2J_dqiy_dy, d2J_dqiz_dy,
				d2J_dpix_dz, d2J_dpiy_dz, d2J_dpiz_dz, d2J_dqix_dz, d2J_dqiy_dz, d2J_dqiz_dz,
				d2J_dpix_da, d2J_dpiy_da, d2J_dpiz_da, d2J_dqix_da, d2J_dqiy_da, d2J_dqiz_da,
				d2J_dpix_db, d2J_dpiy_db, d2J_dpiz_db, d2J_dqix_db, d2J_dqiy_db, d2J_dqiz_db,
				d2J_dpix_dc, d2J_dpiy_dc, d2J_dpiz_dc, d2J_dqix_dc, d2J_dqiy_dc, d2J_dqiz_dc;

		d2J_dpix_dx = 2 * cos_yaw * cos_pitch;
		d2J_dpix_dy = 2 * cos_pitch * sin_yaw;
		d2J_dpix_dz = -2 * sin_pitch;
		d2J_dpix_da = cos_pitch * sin_yaw * (2 * piz * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) - 2 * piy * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll) + 2 * pix * cos_yaw * cos_pitch) - cos_yaw * cos_pitch * (2 * piy * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll) - 2 * piz * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + 2 * pix * cos_pitch * sin_yaw) - 2 * cos_pitch * sin_yaw * (correction_x - qix - piy * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll) + piz * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) + pix * cos_yaw * cos_pitch) + 2 * cos_yaw * cos_pitch * (correction_y - qiy + piy * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll) - piz * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + pix * cos_pitch * sin_yaw);
		d2J_dpix_db = sin_pitch * (2 * pix * cos_pitch + 2 * piz * cos_roll * sin_pitch + 2 * piy * sin_pitch * sin_roll) - 2 * cos_pitch * (correction_z - qiz - pix * sin_pitch + piz * cos_pitch * cos_roll + piy * cos_pitch * sin_roll) + cos_yaw * cos_pitch * (2 * piz * cos_yaw * cos_pitch * cos_roll - 2 * pix * cos_yaw * sin_pitch + 2 * piy * cos_yaw * cos_pitch * sin_roll) - 2 * sin_yaw * sin_pitch * (correction_y - qiy + piy * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll) - piz * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + pix * cos_pitch * sin_yaw) + cos_pitch * sin_yaw * (2 * piz * cos_pitch * cos_roll * sin_yaw - 2 * pix * sin_yaw * sin_pitch + 2 * piy * cos_pitch * sin_yaw * sin_roll) - 2 * cos_yaw * sin_pitch * (correction_x - qix - piy * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll) + piz * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) + pix * cos_yaw * cos_pitch);
		d2J_dpix_dc = cos_yaw * cos_pitch * (2 * piy * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) + 2 * piz * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll)) - sin_pitch * (2 * piy * cos_pitch * cos_roll - 2 * piz * cos_pitch * sin_roll) - cos_pitch * sin_yaw * (2 * piy * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + 2 * piz * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll));
		d2J_dpiy_dx = 2 * cos_yaw * sin_pitch * sin_roll - 2 * cos_roll * sin_yaw;
		d2J_dpiy_dy = 2 * cos_yaw * cos_roll + 2 * sin_yaw * sin_pitch * sin_roll;
		d2J_dpiy_dz = 2 * cos_pitch * sin_roll;
		d2J_dpiy_da = (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll) * (2 * piz * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) - 2 * piy * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll) + 2 * pix * cos_yaw * cos_pitch) + (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll) * (2 * piy * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll) - 2 * piz * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + 2 * pix * cos_pitch * sin_yaw) - (2 * cos_yaw * cos_roll + 2 * sin_yaw * sin_pitch * sin_roll) * (correction_x - qix - piy * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll) + piz * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) + pix * cos_yaw * cos_pitch) - (2 * cos_roll * sin_yaw - 2 * cos_yaw * sin_pitch * sin_roll) * (correction_y - qiy + piy * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll) - piz * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + pix * cos_pitch * sin_yaw);
		d2J_dpiy_db = (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll) * (2 * piz * cos_pitch * cos_roll * sin_yaw - 2 * pix * sin_yaw * sin_pitch + 2 * piy * cos_pitch * sin_yaw * sin_roll) - (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll) * (2 * piz * cos_yaw * cos_pitch * cos_roll - 2 * pix * cos_yaw * sin_pitch + 2 * piy * cos_yaw * cos_pitch * sin_roll) - 2 * sin_pitch * sin_roll * (correction_z - qiz - pix * sin_pitch + piz * cos_pitch * cos_roll + piy * cos_pitch * sin_roll) - cos_pitch * sin_roll * (2 * pix * cos_pitch + 2 * piz * cos_roll * sin_pitch + 2 * piy * sin_pitch * sin_roll) + 2 * cos_yaw * cos_pitch * sin_roll * (correction_x - qix - piy * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll) + piz * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) + pix * cos_yaw * cos_pitch) + 2 * cos_pitch * sin_yaw * sin_roll * (correction_y - qiy + piy * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll) - piz * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + pix * cos_pitch * sin_yaw);
		d2J_dpiy_dc = (2 * sin_yaw * sin_roll + 2 * cos_yaw * cos_roll * sin_pitch) * (correction_x - qix - piy * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll) + piz * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) + pix * cos_yaw * cos_pitch) - (2 * cos_yaw * sin_roll - 2 * cos_roll * sin_yaw * sin_pitch) * (correction_y - qiy + piy * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll) - piz * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + pix * cos_pitch * sin_yaw) - (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll) * (2 * piy * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + 2 * piz * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll)) - (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll) * (2 * piy * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) + 2 * piz * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll)) + 2 * cos_pitch * cos_roll * (correction_z - qiz - pix * sin_pitch + piz * cos_pitch * cos_roll + piy * cos_pitch * sin_roll) + cos_pitch * sin_roll * (2 * piy * cos_pitch * cos_roll - 2 * piz * cos_pitch * sin_roll);
		d2J_dpiz_dx = 2 * sin_yaw * sin_roll + 2 * cos_yaw * cos_roll * sin_pitch;
		d2J_dpiz_dy = 2 * cos_roll * sin_yaw * sin_pitch - 2 * cos_yaw * sin_roll;
		d2J_dpiz_dz = 2 * cos_pitch * cos_roll;
		d2J_dpiz_da = (2 * cos_yaw * sin_roll - 2 * cos_roll * sin_yaw * sin_pitch) * (correction_x - qix - piy * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll) + piz * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) + pix * cos_yaw * cos_pitch) - (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) * (2 * piy * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll) - 2 * piz * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + 2 * pix * cos_pitch * sin_yaw) - (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) * (2 * piz * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) - 2 * piy * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll) + 2 * pix * cos_yaw * cos_pitch) + (2 * sin_yaw * sin_roll + 2 * cos_yaw * cos_roll * sin_pitch) * (correction_y - qiy + piy * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll) - piz * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + pix * cos_pitch * sin_yaw);
		d2J_dpiz_db = (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) * (2 * piz * cos_yaw * cos_pitch * cos_roll - 2 * pix * cos_yaw * sin_pitch + 2 * piy * cos_yaw * cos_pitch * sin_roll) - (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) * (2 * piz * cos_pitch * cos_roll * sin_yaw - 2 * pix * sin_yaw * sin_pitch + 2 * piy * cos_pitch * sin_yaw * sin_roll) - 2 * cos_roll * sin_pitch * (correction_z - qiz - pix * sin_pitch + piz * cos_pitch * cos_roll + piy * cos_pitch * sin_roll) - cos_pitch * cos_roll * (2 * pix * cos_pitch + 2 * piz * cos_roll * sin_pitch + 2 * piy * sin_pitch * sin_roll) + 2 * cos_yaw * cos_pitch * cos_roll * (correction_x - qix - piy * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll) + piz * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) + pix * cos_yaw * cos_pitch) + 2 * cos_pitch * cos_roll * sin_yaw * (correction_y - qiy + piy * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll) - piz * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + pix * cos_pitch * sin_yaw);
		d2J_dpiz_dc = (2 * cos_roll * sin_yaw - 2 * cos_yaw * sin_pitch * sin_roll) * (correction_x - qix - piy * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll) + piz * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) + pix * cos_yaw * cos_pitch) - (2 * cos_yaw * cos_roll + 2 * sin_yaw * sin_pitch * sin_roll) * (correction_y - qiy + piy * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll) - piz * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + pix * cos_pitch * sin_yaw) + (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) * (2 * piy * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) + 2 * piz * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll)) + (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) * (2 * piy * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + 2 * piz * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll)) + cos_pitch * cos_roll * (2 * piy * cos_pitch * cos_roll - 2 * piz * cos_pitch * sin_roll) - 2 * cos_pitch * sin_roll * (correction_z - qiz - pix * sin_pitch + piz * cos_pitch * cos_roll + piy * cos_pitch * sin_roll);
		d2J_dqix_dx = -2;
		d2J_dqix_dy = 0;
		d2J_dqix_dz = 0;
		d2J_dqix_da = 2 * piy * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll) - 2 * piz * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + 2 * pix * cos_pitch * sin_yaw;
		d2J_dqix_db = 2 * pix * cos_yaw * sin_pitch - 2 * piz * cos_yaw * cos_pitch * cos_roll - 2 * piy * cos_yaw * cos_pitch * sin_roll;
		d2J_dqix_dc = -2 * piy * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) - 2 * piz * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll);
		d2J_dqiy_dx = 0;
		d2J_dqiy_dy = -2;
		d2J_dqiy_dz = 0;
		d2J_dqiy_da = 2 * piy * (cos_roll * sin_yaw - cos_yaw * sin_pitch * sin_roll) - 2 * piz * (sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch) - 2 * pix * cos_yaw * cos_pitch;
		d2J_dqiy_db = 2 * pix * sin_yaw * sin_pitch - 2 * piz * cos_pitch * cos_roll * sin_yaw - 2 * piy * cos_pitch * sin_yaw * sin_roll;
		d2J_dqiy_dc = 2 * piy * (cos_yaw * sin_roll - cos_roll * sin_yaw * sin_pitch) + 2 * piz * (cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll);
		d2J_dqiz_dx = 0;
		d2J_dqiz_dy = 0;
		d2J_dqiz_dz = -2;
		d2J_dqiz_da = 0;
		d2J_dqiz_db = 2 * pix * cos_pitch + 2 * piz * cos_roll * sin_pitch + 2 * piy * sin_pitch * sin_roll;
		d2J_dqiz_dc = 2 * piz * cos_pitch * sin_roll - 2 * piy * cos_pitch * cos_roll;
		d2J_dZdX_temp << d2J_dpix_dx, d2J_dpiy_dx, d2J_dpiz_dx, d2J_dqix_dx, d2J_dqiy_dx, d2J_dqiz_dx, d2J_dpix_dy, d2J_dpiy_dy, d2J_dpiz_dy, d2J_dqix_dy, d2J_dqiy_dy, d2J_dqiz_dy, d2J_dpix_dz, d2J_dpiy_dz, d2J_dpiz_dz, d2J_dqix_dz, d2J_dqiy_dz, d2J_dqiz_dz, d2J_dpix_da, d2J_dpiy_da, d2J_dpiz_da, d2J_dqix_da, d2J_dqiy_da, d2J_dqiz_da, d2J_dpix_db, d2J_dpiy_db, d2J_dpiz_db, d2J_dqix_db, d2J_dqiy_db, d2J_dqiz_db, d2J_dpix_dc, d2J_dpiy_dc, d2J_dpiz_dc, d2J_dqix_dc, d2J_dqiy_dc, d2J_dqiz_dc;
		d2J_dZdX.block<6, 6>(0, 6 * k) = d2J_dZdX_temp;
	}

	Eigen::MatrixXd cov_z(6 * ambient_cloud_orrespondences_size, 6 * ambient_cloud_orrespondences_size);
	cov_z = sensor_std_dev_noise * sensor_std_dev_noise * Eigen::MatrixXd::Identity(6 * ambient_cloud_orrespondences_size, 6 * ambient_cloud_orrespondences_size);
	Eigen::FullPivLU<Eigen::MatrixXd> lu(d2J_dX2);
	Eigen::MatrixXd d2J_dX2_inverse = lu.inverse();
	covariance_out = d2J_dX2_inverse * d2J_dZdX * cov_z * d2J_dZdX.transpose() * d2J_dX2_inverse;

	return true;
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </RegistrationCovariancePointToPoint3D-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <template instantiations>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </template instantiations>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

} /* namespace dynamic_robot_localization */
