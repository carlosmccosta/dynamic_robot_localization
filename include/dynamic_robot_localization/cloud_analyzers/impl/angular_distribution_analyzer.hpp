/**\file angular_distribution_analyzer.hpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/cloud_analyzers/angular_distribution_analyzer.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


namespace dynamic_robot_localization {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
AngularDistributionAnalyzer<PointT>::AngularDistributionAnalyzer() :
		number_of_angular_bins_(180) {}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <AngularDistributionAnalyzer-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
void AngularDistributionAnalyzer<PointT>::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace) {
	std::string final_param_name;
	if (ros::param::search(private_node_handle->getNamespace() + "/" + configuration_namespace, "number_of_angular_bins", final_param_name)) {
		private_node_handle->param(final_param_name, number_of_angular_bins_, 180);
	}
}


template<typename PointT>
bool AngularDistributionAnalyzer<PointT>::computeAnalysisHistogram(const tf2::Transform& estimated_pose, const pcl::PointCloud<PointT>& pointcloud, std::vector<size_t>& analysis_histogram_out) {
	if (pointcloud.empty() || (analysis_histogram_out.empty() && number_of_angular_bins_ == 0)) {
		return false;
	}

	if (analysis_histogram_out.empty()) {
		analysis_histogram_out.resize(number_of_angular_bins_, 0);
	} else {
		analysis_histogram_out.assign(analysis_histogram_out.size(), 0);
	}

	double analysis_histogram_out_size = analysis_histogram_out.size();
	tf2::Vector3 estimated_pose_position = estimated_pose.getOrigin();
	tf2::Vector3 estimated_pose_orientation = tf2::quatRotate(estimated_pose.getRotation().normalize(), tf2::Vector3(1,0,0)).normalize();
	tf2::Vector3 reference_normal_vector = estimated_pose_orientation.cross(tf2::Vector3(pointcloud[0].x - estimated_pose_position.x(), pointcloud[0].y - estimated_pose_position.y(), pointcloud[0].z - estimated_pose_position.z())).normalize();

	for (size_t i = 0; i < pointcloud.size(); ++i) {
		const PointT& current_point = pointcloud[i];
		tf2::Vector3 current_point_centered_on_estimated_pose(
				current_point.x - estimated_pose_position.x(),
				current_point.y - estimated_pose_position.y(),
				current_point.z - estimated_pose_position.z());

		if (!(current_point_centered_on_estimated_pose.x() == 0.0f && current_point_centered_on_estimated_pose.y() == 0.0f && current_point_centered_on_estimated_pose.z() == 0.0f)) {
			current_point_centered_on_estimated_pose.normalize();
			double dot_product = estimated_pose_orientation.dot(current_point_centered_on_estimated_pose); // cos(angle) = (a.b) / (|a|*|b|)
			tf2::Vector3 cross_product = estimated_pose_orientation.cross(current_point_centered_on_estimated_pose);

			// cos range -> [-1..1]
			// change range of cos to [-2..0] and then remap values to have range [0..1] (even though they can only have values in [0..0.5], the remaining [0.5..1] are given when the normal vector is in the opposite side of the reference normal vector
			size_t bin_position = (size_t)(((dot_product - 1.0) * analysis_histogram_out_size) / -4.0); // size_t conversion truncates value (same as std::trunc)
			if (reference_normal_vector.dot(cross_product) < 0.0) {
				bin_position = analysis_histogram_out.size() - bin_position - 1;
			}

			// the first and last bin have the points directly in front of the robot
			// the direction is given by the first point, that will fix the reference_normal_vector (rotation vector in a right hand coordinate system)
			if (bin_position < analysis_histogram_out.size()) {
				++analysis_histogram_out[bin_position];
			}
		}
	}

	return true;
}


template<typename PointT>
double AngularDistributionAnalyzer<PointT>::computeAnalysis(std::vector<size_t>& analysis_histogram) {
	if (analysis_histogram.empty()) { return -1.0; }

	size_t filled_bins = 0;
	for (size_t i = 0; i < analysis_histogram.size(); ++i) {
		if (analysis_histogram[i] > 0) {
			++filled_bins;
		}
	}

	return (double)filled_bins / (double)analysis_histogram.size();
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </AngularDistributionAnalyzer-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <template instantiations>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </template instantiations>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

} /* namespace dynamic_robot_localization */
