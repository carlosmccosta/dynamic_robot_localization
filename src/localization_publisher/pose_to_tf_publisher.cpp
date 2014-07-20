/**\file pose_to_tf_publisher.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/localization_publisher/pose_to_tf_publisher.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
PoseToTFPublisher::PoseToTFPublisher() :
		publish_rate_(100), number_tfs_published_(0) {
}

PoseToTFPublisher::~PoseToTFPublisher() {}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <ros integration functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void PoseToTFPublisher::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle) {
	node_handle_ = node_handle;
	private_node_handle_ = private_node_handle;

	private_node_handle_->param("publish_rate", publish_rate_, 100.0);

	private_node_handle_->param("initial_pose_topic", initial_pose_topic_, std::string(""));
	private_node_handle_->param("initial_pose_with_covariance_stamped_topic", initial_pose_with_covariance_stamped_topic_, std::string("initialpose"));
	private_node_handle_->param("map_frame_id", map_frame_id_, std::string("map"));
	private_node_handle_->param("odom_frame_id", odom_frame_id_, std::string("odom"));
	private_node_handle_->param("base_link_frame_id", base_link_frame_id_, std::string("base_link"));

	transform_stamped_map_to_odom_.child_frame_id = odom_frame_id_;
	transform_stamped_map_to_odom_.header.frame_id = map_frame_id_;
}


void PoseToTFPublisher::publishInitialPoseFromParameterServer() {
	// initial pose tf
	double roll, pitch, yaw, x, y, z;
	private_node_handle_->param("initial_x", x, 0.0);
	private_node_handle_->param("initial_y", y, 0.0);
	private_node_handle_->param("initial_z", z, 0.0);
	private_node_handle_->param("initial_roll", roll, 0.0);
	private_node_handle_->param("initial_pitch", pitch, 0.0);
	private_node_handle_->param("initial_yaw", yaw, 0.0);
	publishTFMapToBaseLinkFromInitialPose(x, y, z, roll, pitch, yaw);
}


void PoseToTFPublisher::startPublishingTFFromPoseTopics() {
	if (!initial_pose_topic_.empty()) {
		initial_pose_subscriber_ = node_handle_->subscribe(initial_pose_topic_, 5, &dynamic_robot_localization::PoseToTFPublisher::publishTFMapToOdomFromPose, this);
	}

	if (!initial_pose_with_covariance_stamped_topic_.empty()) {
		initial_pose_with_covariance_stamped_subscriber_ = node_handle_->subscribe(initial_pose_with_covariance_stamped_topic_, 5,
		        &dynamic_robot_localization::PoseToTFPublisher::publishTFMapToOdomFromPoseWithCovarianceStamped, this);
	}

	ros::Rate publish_rate(publish_rate_);
	while (ros::ok()) {
		publishTFMapToOdom();
		publish_rate.sleep();
		ros::spinOnce();
	}
}


void PoseToTFPublisher::stopPublishingTFFromPoseTopics() {
	if (!initial_pose_topic_.empty()) {
		initial_pose_subscriber_.shutdown();
	}

	if (!initial_pose_with_covariance_stamped_topic_.empty()) {
		initial_pose_with_covariance_stamped_subscriber_.shutdown();
	}
}


void PoseToTFPublisher::publishTFMapToOdomFromPose(const geometry_msgs::PoseConstPtr& pose) {
	tf2::Transform transform_pose(
			tf2::Quaternion(pose->orientation.x, pose->orientation.y, pose->orientation.z, pose->orientation.w),
			tf2::Vector3(pose->position.x, pose->position.y, pose->position.z));

	publishTFMapToOdom(transform_pose);
}


void PoseToTFPublisher::publishTFMapToOdomFromPoseWithCovarianceStamped(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose) {
	if (pose->header.frame_id.empty()) {
		return;
	}

	tf2::Transform transform_pose(tf2::Quaternion(pose->pose.pose.orientation.x, pose->pose.pose.orientation.y, pose->pose.pose.orientation.z, pose->pose.pose.orientation.w),
	        tf2::Vector3(pose->pose.pose.position.x, pose->pose.pose.position.y, pose->pose.pose.position.z));

	if (pose->header.frame_id != map_frame_id_) {
		// transform to map (global frame reference)
		tf2::Transform transform_pose_to_map;
		if (!tf_collector_.lookForTransform(transform_pose_to_map, map_frame_id_, ros::Time::now(), pose->header.frame_id, pose->header.stamp, map_frame_id_)) {
			return;
		}

		transform_pose *= transform_pose_to_map;
	}

//	if (pose->header.stamp <= ros::Time::now()) { // some localization methods publish pose in the near future
//		addOdometryDisplacementToTransform(transform_pose, pose->header.stamp);
//	}
	publishTFMapToOdom(transform_pose, pose->header.stamp);
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </ros integration functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <tf update functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
bool PoseToTFPublisher::addOdometryDisplacementToTransform(tf2::Transform& transform, const ros::Time& time_of_transform, const ros::Time& target_time) {
	tf2::Transform odometry_tf_from_pose_time_to_now;
	if (tf_collector_.lookForTransform(odometry_tf_from_pose_time_to_now, base_link_frame_id_, target_time, base_link_frame_id_, time_of_transform, map_frame_id_)) {
		// include odometry displacement from pose publish time to current time
		transform = transform * odometry_tf_from_pose_time_to_now.inverse();
		return true;
	}

	return false;
}


void PoseToTFPublisher::publishTFMapToOdom() {
	transform_stamped_map_to_odom_.header.seq = number_tfs_published_++;
	transform_stamped_map_to_odom_.header.stamp = ros::Time::now();
	transform_broadcaster_.sendTransform(transform_stamped_map_to_odom_);
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </tf update functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <pose to tf functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void PoseToTFPublisher::publishTFMapToBaseLinkFromInitialPose(double x, double y, double z, double roll, double pitch, double yaw) {
	tf2::Quaternion orientation;
	orientation.setRPY(roll, pitch, yaw);

	transform_map_to_odom_ = tf2::Transform(orientation, tf2::Vector3(x, y, z));
	laserscan_to_pointcloud::tf_rosmsg_eigen_conversions::transformTF2ToMsg(transform_map_to_odom_, transform_stamped_map_to_odom_.transform);
	publishTFMapToOdom();
}


void PoseToTFPublisher::publishTFMapToOdomFromGlobalPose(double x, double y, double z, double roll, double pitch, double yaw) {
	tf2::Quaternion orientation;
	orientation.setRPY(roll, pitch, yaw);
	publishTFMapToOdom(tf2::Transform(orientation, tf2::Vector3(x, y, z)));
}


void PoseToTFPublisher::publishTFMapToOdom(const tf2::Transform& transform_map_to_base_link, ros::Time tf_time) {
	tf2::Transform transform_odom_to_base_link;
	if (!tf_collector_.lookForTransform(transform_odom_to_base_link, base_link_frame_id_, odom_frame_id_, tf_time)) {
		return;
	}

	// map_to_base = map_to_odom * odom_to_base
	// map_to_odom = map_to_base * base_to_odom)
	transform_map_to_odom_ = transform_map_to_base_link * transform_odom_to_base_link;
	laserscan_to_pointcloud::tf_rosmsg_eigen_conversions::transformTF2ToMsg(transform_map_to_odom_, transform_stamped_map_to_odom_.transform);
	publishTFMapToOdom();
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </pose to tf functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// =============================================================================   <private-section>   =========================================================================
// =============================================================================   </private-section>  =========================================================================

} /* namespace dynamic_robot_localization */

