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
		publish_rate_(100),
		publish_last_pose_tf_timeout_seconds_(-1.0),
		last_pose_time_(0),
		invert_tf_transform_(false),
		invert_tf_hierarchy_(false),
		transform_pose_to_map_frame_id_(true),
		number_tfs_published_(0) {
}

PoseToTFPublisher::~PoseToTFPublisher() {}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <ros integration functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void PoseToTFPublisher::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace) {
	node_handle_ = node_handle;
	private_node_handle_ = private_node_handle;

	private_node_handle_->param("publish_rate", publish_rate_, 100.0);
	private_node_handle_->param("publish_last_pose_tf_timeout_seconds", publish_last_pose_tf_timeout_seconds_, -1.0);

	private_node_handle_->param("pose_stamped_topic", pose_stamped_topic_, std::string(""));
	private_node_handle_->param("pose_with_covariance_stamped_topic", pose_with_covariance_stamped_topic_, std::string("/initialpose"));
	private_node_handle_->param("odometry_topic", odometry_topic_, std::string(""));
	private_node_handle_->param("map_frame_id", map_frame_id_, std::string("map"));
	private_node_handle_->param("odom_frame_id", odom_frame_id_, std::string("odom"));
	private_node_handle_->param("base_link_frame_id", base_link_frame_id_, std::string("base_link"));

	private_node_handle_->param("invert_tf_transform", invert_tf_transform_, false);
	private_node_handle_->param("invert_tf_hierarchy", invert_tf_hierarchy_, false);
	private_node_handle_->param("transform_pose_to_map_frame_id", transform_pose_to_map_frame_id_, true);

	transform_stamped_map_to_odom_.header.frame_id = invert_tf_hierarchy_ ? odom_frame_id_ : map_frame_id_;
	transform_stamped_map_to_odom_.child_frame_id = invert_tf_hierarchy_ ? map_frame_id_ : odom_frame_id_;
	transform_stamped_map_to_odom_.transform.translation.x = 0.0;
	transform_stamped_map_to_odom_.transform.translation.y = 0.0;
	transform_stamped_map_to_odom_.transform.translation.z = 0.0;
	transform_stamped_map_to_odom_.transform.rotation.x = 0.0;
	transform_stamped_map_to_odom_.transform.rotation.y = 0.0;
	transform_stamped_map_to_odom_.transform.rotation.z = 0.0;
	transform_stamped_map_to_odom_.transform.rotation.w = 1.0;
}


void PoseToTFPublisher::publishInitialPoseFromParameterServer() {
	// initial pose tf
	double x, y, z, roll, pitch, yaw;
	bool initial_pose_in_map_to_base;
	private_node_handle_->param("initial_pose_in_map_to_base", initial_pose_in_map_to_base, true);
	private_node_handle_->param("initial_x", x, 0.0);
	private_node_handle_->param("initial_y", y, 0.0);
	private_node_handle_->param("initial_z", z, 0.0);
	private_node_handle_->param("initial_roll", roll, 0.0);
	private_node_handle_->param("initial_pitch", pitch, 0.0);
	private_node_handle_->param("initial_yaw", yaw, 0.0);

	if (initial_pose_in_map_to_base) {
		publishTFMapToOdomFromMapToBasePose(x, y, z, roll, pitch, yaw);
	} else {
		publishTFMapToOdomFromMapToOdomPose(x, y, z, roll, pitch, yaw);
	}
}


void PoseToTFPublisher::startPublishingTFFromPoseTopics() {
	if (!pose_stamped_topic_.empty()) {
		pose_stamped_subscriber_ = node_handle_->subscribe(pose_stamped_topic_, 5, &dynamic_robot_localization::PoseToTFPublisher::publishTFMapToOdomFromPoseStamped, this);
	}

	if (!pose_with_covariance_stamped_topic_.empty()) {
		pose_with_covariance_stamped_subscriber_ = node_handle_->subscribe(pose_with_covariance_stamped_topic_, 5,
				&dynamic_robot_localization::PoseToTFPublisher::publishTFMapToOdomFromPoseWithCovarianceStamped, this);
	}

	if (!odometry_topic_.empty()) {
		odometry_subscriber_ = node_handle_->subscribe(odometry_topic_, 5, &dynamic_robot_localization::PoseToTFPublisher::publishTFMapToOdomFromOdometry, this);
	}

	ros::Rate publish_rate(publish_rate_);
	while (ros::ok()) {
		publishTFMapToOdom();
		publish_rate.sleep();
		ros::spinOnce();
	}
}


void PoseToTFPublisher::stopPublishingTFFromPoseTopics() {
	if (!pose_stamped_topic_.empty()) {
		pose_stamped_subscriber_.shutdown();
	}

	if (!pose_with_covariance_stamped_topic_.empty()) {
		pose_with_covariance_stamped_subscriber_.shutdown();
	}

	if (!odometry_topic_.empty()) {
		odometry_subscriber_.shutdown();
	}
}


void PoseToTFPublisher::publishTFMapToOdomFromPoseStamped(const geometry_msgs::PoseStampedConstPtr& pose) {
	if (pose->header.frame_id.empty()) {
		return;
	}

	tf2::Transform transform_pose(
			tf2::Quaternion(pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z, pose->pose.orientation.w),
			tf2::Vector3(pose->pose.position.x, pose->pose.position.y, pose->pose.position.z));

	if (transform_pose_to_map_frame_id_ && pose->header.frame_id != map_frame_id_) {
		// transform to map (global frame reference)
		tf2::Transform transform_pose_to_map;
		if (!tf_collector_.lookForTransform(transform_pose_to_map, map_frame_id_, pose->header.frame_id, pose->header.stamp)) {
			return;
		}

		transform_pose *= transform_pose_to_map;
		ROS_INFO_STREAM("Pose received in " << pose->header.frame_id << " frame instead of " << map_frame_id_);
	}

	publishTFMapToOdom(transform_pose, pose->header.stamp);
}


void PoseToTFPublisher::publishTFMapToOdomFromPoseWithCovarianceStamped(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose) {
	geometry_msgs::PoseStampedPtr poseStamped(new geometry_msgs::PoseStamped());
	poseStamped->header = pose->header;
	poseStamped->pose = pose->pose.pose;

	publishTFMapToOdomFromPoseStamped(poseStamped);
}


void PoseToTFPublisher::publishTFMapToOdomFromOdometry(const nav_msgs::OdometryConstPtr& odom) {
	geometry_msgs::PoseStampedPtr poseStamped(new geometry_msgs::PoseStamped());
	poseStamped->header = odom->header;
	poseStamped->pose = odom->pose.pose;

	publishTFMapToOdomFromPoseStamped(poseStamped);
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


bool PoseToTFPublisher::publishTFMapToOdom(bool check_pose_timeout) {
	ros::Time current_time = ros::Time::now();
	if (!check_pose_timeout || publish_last_pose_tf_timeout_seconds_ <= 0.0 || (current_time - last_pose_time_).toSec() <= publish_last_pose_tf_timeout_seconds_ || last_pose_time_.toSec() < 1.0) {
		transform_stamped_map_to_odom_.header.seq = number_tfs_published_++;
		transform_stamped_map_to_odom_.header.stamp = current_time;
		transform_broadcaster_.sendTransform(transform_stamped_map_to_odom_);
		return true;
	}

	ROS_WARN_STREAM_THROTTLE(1.0, "Pose to TF publisher reached timeout for last valid pose" \
			<< "\n\tTF translation -> [ x: " << transform_stamped_map_to_odom_.transform.translation.x << " | y: " << transform_stamped_map_to_odom_.transform.translation.y << " | z: " << transform_stamped_map_to_odom_.transform.translation.z << " ]" \
			<< "\n\tTF orientation -> [ qx: " << transform_stamped_map_to_odom_.transform.rotation.x << " | qy: " << transform_stamped_map_to_odom_.transform.rotation.y << " | qz: " << transform_stamped_map_to_odom_.transform.rotation.z << " | qw: " << transform_stamped_map_to_odom_.transform.rotation.w << " ]" \
			<< "\n\tCurrent time: " << current_time \
			<< "\n\tLast pose time: " << last_pose_time_);
	return false;
}

bool PoseToTFPublisher::updateTFMessage(tf2::Transform& transform) {
	if (boost::math::isfinite(transform.getOrigin().getX()) &&
		boost::math::isfinite(transform.getOrigin().getY()) &&
		boost::math::isfinite(transform.getOrigin().getZ()) &&
		boost::math::isfinite(transform.getRotation().getX()) &&
		boost::math::isfinite(transform.getRotation().getY()) &&
		boost::math::isfinite(transform.getRotation().getZ()) &&
		boost::math::isfinite(transform.getRotation().getW())) {

		if (transform.getRotation().getX() == 0.0 && transform.getRotation().getY() == 0.0 && transform.getRotation().getZ() == 0.0 && transform.getRotation().getW() == 0.0) { return false; }
		transform.getRotation().normalize();
		laserscan_to_pointcloud::tf_rosmsg_eigen_conversions::transformTF2ToMsg(transform, transform_stamped_map_to_odom_.transform);
		ROS_DEBUG_STREAM("Updating TF between " << transform_stamped_map_to_odom_.header.frame_id << " and " << transform_stamped_map_to_odom_.child_frame_id \
				<< "\n\tTF translation -> [ x: " << transform_stamped_map_to_odom_.transform.translation.x << " | y: " << transform_stamped_map_to_odom_.transform.translation.y << " | z: " << transform_stamped_map_to_odom_.transform.translation.z << " ]" \
				<< "\n\tTF orientation -> [ qx: " << transform_stamped_map_to_odom_.transform.rotation.x << " | qy: " << transform_stamped_map_to_odom_.transform.rotation.y << " | qz: " << transform_stamped_map_to_odom_.transform.rotation.z << " | qw: " << transform_stamped_map_to_odom_.transform.rotation.w << " ]");
		return true;
	}

	return false;
}


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </tf update functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <pose to tf functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void PoseToTFPublisher::publishTFMapToOdomFromMapToOdomPose(double x, double y, double z, double roll, double pitch, double yaw) {
	tf2::Quaternion orientation;
	orientation.setRPY(roll, pitch, yaw);

	transform_map_to_odom_ = tf2::Transform(orientation, tf2::Vector3(x, y, z));

	if (invert_tf_transform_) {
		transform_map_to_odom_ = transform_map_to_odom_.inverse();
	}

	updateTFMessage(transform_map_to_odom_);
	last_pose_time_ = ros::Time::now();
	publishTFMapToOdom(false);

	ROS_INFO_STREAM("Published global pose from map to odom estimate [ x: " << x << ", y: " << y << ", z: " << z
			<< " | r: " << roll << ", p: " << pitch << ", y: " << yaw
			<< " | qx: " << orientation.x() << ", qy: " << orientation.y() << ", qz: " << orientation.z() << ", qw: " << orientation.w() << " ]");
}


void PoseToTFPublisher::publishTFMapToOdomFromMapToBasePose(double x, double y, double z, double roll, double pitch, double yaw) {
	tf2::Quaternion orientation;
	orientation.setRPY(roll, pitch, yaw);
	tf2::Transform transform(orientation, tf2::Vector3(x, y, z));

	transform_map_to_odom_ = transform;
	last_pose_time_ = ros::Time::now();
	updateTFMessage(transform_map_to_odom_);

	if (publishTFMapToOdom(transform, ros::Time::now(), ros::Duration(5)), false) {
		ROS_INFO_STREAM("Published global pose from map to base estimate [ x: " << x << ", y: " << y << ", z: " << z \
				<< " | r: " << roll << ", p: " << pitch << ", y: " << yaw \
				<< " | qx: " << orientation.x() << ", qy: " << orientation.y() << ", qz: " << orientation.z() << ", qw: " << orientation.w() << " ]");
	} else {
		ros::Time end_time = ros::Time::now() + ros::Duration(10);
		ros::Duration wait_duration(0.005);

		while (ros::Time::now() < end_time) {
			if (publishTFMapToOdom(transform, ros::Time::now(), ros::Duration(0.1), false)) {
				ROS_INFO_STREAM("Published global pose from map to base estimate [ x: " << x << ", y: " << y << ", z: " << z \
						<< " || r: " << roll << ", p: " << pitch << ", y: " << yaw \
						<< " || qx: " << orientation.x() << ", qy: " << orientation.y() << ", qz: " << orientation.z() << ", qw: " << orientation.w() << " ]");
				return;
			}
			wait_duration.sleep();
		}

		ROS_WARN_STREAM("Failed to find TF between " << odom_frame_id_ << " and " << base_link_frame_id_ << " when setting initial pose");
		publishTFMapToOdomFromMapToOdomPose(x, y, z, roll, pitch, yaw);
	}
}


bool PoseToTFPublisher::publishTFMapToOdom(const tf2::Transform& transform_base_link_to_map, ros::Time tf_time, ros::Duration tf_timeout, bool check_pose_timeout) {
	if (tf_time < last_pose_time_) {
		return false;
	}


	if (!base_link_frame_id_.empty() && !odom_frame_id_.empty()) {
		tf2::Transform transform_odom_to_base_link;
		if (!tf_collector_.lookForTransform(transform_odom_to_base_link, base_link_frame_id_, odom_frame_id_, tf_time, tf_timeout)) {
			return false;
		}

		// base_to_map = base_to_odom * odom_to_map
		// odom_to_map = base_to_map * odom_to_base)
		transform_map_to_odom_ = transform_base_link_to_map * transform_odom_to_base_link;
	} else {
		transform_map_to_odom_ = transform_base_link_to_map;
	}

	if (invert_tf_transform_) {
		transform_map_to_odom_ = transform_map_to_odom_.inverse();
	}

	updateTFMessage(transform_map_to_odom_);
	last_pose_time_ = tf_time;
	return publishTFMapToOdom(check_pose_timeout);
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </pose to tf functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// =============================================================================   <private-section>   =========================================================================
// =============================================================================   </private-section>  =========================================================================

} /* namespace dynamic_robot_localization */

