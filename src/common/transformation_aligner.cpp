/**\file transformation_aligner.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/common/transformation_aligner.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <TransformationAligner-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void TransformationAligner::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, const std::string& configuration_namespace) {
	private_node_handle->param(configuration_namespace + "anchor_tf_frame_id", anchor_tf_frame_id_, std::string("camera_link"));
	private_node_handle->param(configuration_namespace + "rotation_alignment_frame_id", rotation_alignment_frame_id_, std::string("reference_for_rotation_alignment"));
	private_node_handle->param(configuration_namespace + "correct_roll", correct_roll_, false);
	private_node_handle->param(configuration_namespace + "correct_pitch", correct_pitch_, false);
	private_node_handle->param(configuration_namespace + "correct_yaw", correct_yaw_, true);
}


bool TransformationAligner::alignTransformation(tf2::Transform& pose_in_target_frame, const ros::Time& transform_time,
																								const std::string& pose_source_tf_frame_id, const std::string& pose_target_tf_frame_id,
																								laserscan_to_pointcloud::TFCollector& tf_collector, const ros::Duration& tf_timeout) {
	bool update_rotation = (correct_roll_ || correct_pitch_ || correct_yaw_);
	if (update_rotation && rotation_alignment_frame_id_.empty()) {
		ROS_WARN("Failed TransformationAligner for rotation because [rotation_alignment_frame_id] is not defined");
		return false;
	}

	if (!update_rotation) {
		return true;
	}

	std::string anchor_tf_frame_id = (anchor_tf_frame_id_.empty() ? pose_target_tf_frame_id : anchor_tf_frame_id_);
	if (anchor_tf_frame_id_.empty()) {
		anchor_tf_frame_id = pose_source_tf_frame_id;
	} else if (anchor_tf_frame_id_ != pose_source_tf_frame_id && anchor_tf_frame_id_ != pose_target_tf_frame_id) {
		ROS_WARN_STREAM("Failed TransformationAligner because [anchor_tf_frame_id] must be either the source or the target frame of the registration (" << pose_source_tf_frame_id << " | " << pose_target_tf_frame_id << ")");
		return false;
	}

	tf2::Transform transform_from_rotation_override_frame_to_anchor_frame;
	if (tf_collector.lookForTransform(transform_from_rotation_override_frame_to_anchor_frame, anchor_tf_frame_id, rotation_alignment_frame_id_, transform_time, tf_timeout)) {
		if (correctRotations(transform_from_rotation_override_frame_to_anchor_frame, pose_in_target_frame, pose_target_tf_frame_id, anchor_tf_frame_id))
			return correctRotations(transform_from_rotation_override_frame_to_anchor_frame, pose_in_target_frame, pose_target_tf_frame_id, anchor_tf_frame_id); // run twice to minimize the effect of numeric errors compounded on the 3 rotation corrections
	} else {
		ROS_WARN_STREAM("Failed TransformationAligner because TF [ " << rotation_alignment_frame_id_ << " -> " << anchor_tf_frame_id << " ] was not available");
	}

	return false;
}


bool TransformationAligner::correctRotations(const tf2::Transform& transform_from_rotation_override_frame_to_anchor_frame, tf2::Transform& pose_in_target_frame,
																						 const std::string& pose_target_tf_frame_id, const std::string& anchor_tf_frame_id) {
	if (correct_roll_) {
		if (!correctRotation(transform_from_rotation_override_frame_to_anchor_frame, pose_in_target_frame, pose_target_tf_frame_id, anchor_tf_frame_id, RotationAxis::Roll)) {
			return false;
		}
	}

	if (correct_pitch_) {
		if (!correctRotation(transform_from_rotation_override_frame_to_anchor_frame, pose_in_target_frame, pose_target_tf_frame_id, anchor_tf_frame_id, RotationAxis::Pitch)) {
			return false;
		}
	}

	if (correct_yaw_) {
		if (!correctRotation(transform_from_rotation_override_frame_to_anchor_frame, pose_in_target_frame, pose_target_tf_frame_id, anchor_tf_frame_id, RotationAxis::Yaw)) {
			return false;
		}
	}

	return true;
}


bool TransformationAligner::correctRotation(
		const tf2::Transform& transform_from_rotation_override_frame_to_anchor_frame, tf2::Transform& pose_in_target_frame,
		const std::string& pose_target_tf_frame_id, const std::string& anchor_tf_frame_id,
		RotationAxis rotation_axis) {

	tf2::Transform transform_from_rotation_override_frame_to_pose_target_frame;
	if (anchor_tf_frame_id == pose_target_tf_frame_id) {
		transform_from_rotation_override_frame_to_pose_target_frame = transform_from_rotation_override_frame_to_anchor_frame;
	} else {
		transform_from_rotation_override_frame_to_pose_target_frame = pose_in_target_frame * transform_from_rotation_override_frame_to_anchor_frame;
	}

	tf2::Matrix3x3 correction_matrix;
	tf2::Vector3 reference_vector_for_alignment_in_target_frame;
	if (rotation_axis == RotationAxis::Roll) {
		if (computeLineIntersectionWithRotationPlane(
				transform_from_rotation_override_frame_to_pose_target_frame,
				tf2::Vector3(1, 0, 0),
				tf2::Vector3(1, 0, 0),
				tf2::Vector3(0, 0, 1),
				reference_vector_for_alignment_in_target_frame)) {

			double correction_angle_radians = -std::atan2(reference_vector_for_alignment_in_target_frame.y(), reference_vector_for_alignment_in_target_frame.z());
			correction_matrix.setRPY(correction_angle_radians, 0, 0);
			ROS_DEBUG_STREAM("TransformationAligner corrected " << (correction_angle_radians * 180.0 / M_PI) << " degrees in roll");
		} else {
			ROS_DEBUG("TransformationAligner line to plane intersection returned NaNs when computing roll correction");
			return false;
		}
	} else if (rotation_axis == RotationAxis::Pitch) {
		if (computeLineIntersectionWithRotationPlane(
				transform_from_rotation_override_frame_to_pose_target_frame,
				tf2::Vector3(0, 1, 0),
				tf2::Vector3(0, 1, 0),
				tf2::Vector3(0, 0, 1),
				reference_vector_for_alignment_in_target_frame)) {

			double correction_angle_radians = std::atan2(reference_vector_for_alignment_in_target_frame.x(), reference_vector_for_alignment_in_target_frame.z());
			correction_matrix.setRPY(0, correction_angle_radians, 0);
			ROS_DEBUG_STREAM("TransformationAligner corrected " << (correction_angle_radians * 180.0 / M_PI) << " degrees in pitch");
		} else {
			ROS_DEBUG("TransformationAligner line to plane intersection returned NaNs when computing pitch correction");
			return false;
		}
	} else {
		if (computeLineIntersectionWithRotationPlane(
				transform_from_rotation_override_frame_to_pose_target_frame,
				tf2::Vector3(0, 0, 1),
				tf2::Vector3(0, 0, 1),
				tf2::Vector3(1, 0, 0),
				reference_vector_for_alignment_in_target_frame)) {

			double correction_angle_radians = std::atan2(reference_vector_for_alignment_in_target_frame.y(), reference_vector_for_alignment_in_target_frame.x());
			correction_matrix.setRPY(0, 0, correction_angle_radians);
			ROS_DEBUG_STREAM("TransformationAligner corrected " << (correction_angle_radians * 180.0 / M_PI) << " degrees in yaw");
		} else {
			ROS_DEBUG("TransformationAligner line to plane intersection returned NaNs when computing yaw correction");
			return false;
		}
	}

	pose_in_target_frame = (pose_in_target_frame.inverse() * tf2::Transform(correction_matrix)).inverse();
	return true;
}


bool TransformationAligner::computeLineIntersectionWithRotationPlane(const tf2::Transform& transform_from_rotation_override_frame_to_pose_target_frame,
																																		 const tf2::Vector3& rotation_axis_in_target_frame,
																																		 const tf2::Vector3& line_direction_for_intersection_with_rotation_plane_in_rotation_override_frame,
																																		 const tf2::Vector3& reference_axis_for_computing_the_rotation_correction_in_the_rotation_override_frame,
																																		 tf2::Vector3& line_intersection_in_target_frame) {
	tf2::Transform transform_from_pose_target_frame_to_rotation_override_frame = transform_from_rotation_override_frame_to_pose_target_frame.inverse();

	// plane setup
	tf2::Vector3 rotation_plane_normal_in_rotation_override_frame = transform_from_pose_target_frame_to_rotation_override_frame.getBasis() * rotation_axis_in_target_frame;
	tf2::Vector3 rotation_plane_origin_in_rotation_override_frame = transform_from_pose_target_frame_to_rotation_override_frame.getOrigin();

	Eigen::Vector3f rotation_plane_normal_in_rotation_override_frame_eigen(
			rotation_plane_normal_in_rotation_override_frame.x(),
			rotation_plane_normal_in_rotation_override_frame.y(),
			rotation_plane_normal_in_rotation_override_frame.z());

	Eigen::Vector3f rotation_plane_origin_in_rotation_override_frame_eigen(
			rotation_plane_origin_in_rotation_override_frame.x(),
			rotation_plane_origin_in_rotation_override_frame.y(),
			rotation_plane_origin_in_rotation_override_frame.z());

	Eigen::Hyperplane<float, 3> rotation_plane(rotation_plane_normal_in_rotation_override_frame_eigen, rotation_plane_origin_in_rotation_override_frame_eigen);

	// line setup
	Eigen::Vector3f reference_axis_for_computing_the_rotation_correction_in_the_rotation_override_frame_eigen(
			reference_axis_for_computing_the_rotation_correction_in_the_rotation_override_frame.x(),
			reference_axis_for_computing_the_rotation_correction_in_the_rotation_override_frame.y(),
			reference_axis_for_computing_the_rotation_correction_in_the_rotation_override_frame.z());

	Eigen::Vector3f line_direction_for_intersection_with_rotation_plane_in_rotation_override_frame_eigen(
			line_direction_for_intersection_with_rotation_plane_in_rotation_override_frame.x(),
			line_direction_for_intersection_with_rotation_plane_in_rotation_override_frame.y(),
			line_direction_for_intersection_with_rotation_plane_in_rotation_override_frame.z());

	// line-plane intersection
	Eigen::Vector3f intersection_line_origin = rotation_plane_origin_in_rotation_override_frame_eigen + reference_axis_for_computing_the_rotation_correction_in_the_rotation_override_frame_eigen;
	Eigen::ParametrizedLine<float, 3> intersection_line(intersection_line_origin, line_direction_for_intersection_with_rotation_plane_in_rotation_override_frame_eigen);
	Eigen::Vector3f line_plane_intersection = intersection_line.intersectionPoint(rotation_plane);

	if (std::isfinite(line_plane_intersection.x()) && std::isfinite(line_plane_intersection.y()) && std::isfinite(line_plane_intersection.z())) {
		ROS_DEBUG_STREAM("TransformationAligner line to plane intersection: [ " << line_plane_intersection.x() << " | " << line_plane_intersection.y() << " | " << line_plane_intersection.z() << " ]");
		line_intersection_in_target_frame = transform_from_rotation_override_frame_to_pose_target_frame * tf2::Vector3(line_plane_intersection.x(), line_plane_intersection.y(), line_plane_intersection.z());
		return true;
	}

	return false;
}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </TransformationAligner-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// =============================================================================   <private-section>   =========================================================================
// =============================================================================   </private-section>  =========================================================================

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <template instantiations>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </template instantiations>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

} /* namespace dynamic_robot_localization */
