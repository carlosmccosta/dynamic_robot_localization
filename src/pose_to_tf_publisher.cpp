/**\file pose_to_tf_publisher.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/pose_to_tf_publisher.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
PoseToTFPublisher::PoseToTFPublisher(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle) :
		number_tfs_published_(0),
		node_handle_(node_handle), private_node_handle_(private_node_handle) {

	private_node_handle_->param("publish_rate", publish_rate_, 100.0);

	private_node_handle_->param("initial_pose_topic", initial_pose_topic_, std::string("initialpose"));
	private_node_handle_->param("map_frame_id", map_frame_id_, std::string("map"));
	private_node_handle_->param("odom_frame_id", odom_frame_id_, std::string("odom"));
	private_node_handle_->param("base_link_frame_id", base_link_frame_id_, std::string("base_link"));

	transform_stamped_map_to_base_link_.child_frame_id = odom_frame_id_;
	transform_stamped_map_to_base_link_.header.frame_id = map_frame_id_;

	// initial pose tf
	double roll, pitch, yaw, x, y, z;
	private_node_handle_->param("initial_x", x, 0.0);
	private_node_handle_->param("initial_y", y, 0.0);
	private_node_handle_->param("initial_z", z, 0.0);
	private_node_handle_->param("initial_roll", roll, 0.0);
	private_node_handle_->param("initial_pitch", pitch, 0.0);
	private_node_handle_->param("initial_yaw", yaw, 0.0);
	publishTFMapToOdomFromPose(x, y, z, roll, pitch, yaw);
}

PoseToTFPublisher::~PoseToTFPublisher() {}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <TFPublisher-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void PoseToTFPublisher::startPublishingTF() {
	initial_pose_subscriber_ = node_handle_->subscribe(initial_pose_topic_, 5, &dynamic_robot_localization::PoseToTFPublisher::publishTFMapToOdomFromPose, this);

	ros::Rate publish_rate(publish_rate_);
	while (ros::ok()) {
		publishTFMapToOdom();
		publish_rate.sleep();
		ros::spinOnce();
	}
}

void PoseToTFPublisher::stopPublishingTF() {
	initial_pose_subscriber_.shutdown();
}

void PoseToTFPublisher::publishTFMapToOdom() {
	transform_stamped_map_to_base_link_.header.seq = number_tfs_published_++;
	transform_stamped_map_to_base_link_.header.stamp = ros::Time::now();
	transform_broadcaster_.sendTransform(transform_stamped_map_to_base_link_);
}


void PoseToTFPublisher::publishTFMapToOdomFromPose(double x, double y, double z, double roll, double pitch, double yaw) {
	tf2::Quaternion orientation;
	orientation.setRPY(roll, pitch, yaw);
	publishTFMapToOdom(tf2::Transform(orientation, tf2::Vector3(x, y, z)));
}


void PoseToTFPublisher::publishTFMapToOdomFromPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose) {
	tf2::Transform transform_map_to_base_link(
			tf2::Quaternion(pose->pose.pose.orientation.x, pose->pose.pose.orientation.y, pose->pose.pose.orientation.z, pose->pose.pose.orientation.w),
			tf2::Vector3(pose->pose.pose.position.x, pose->pose.pose.position.y, pose->pose.pose.position.z));

	publishTFMapToOdom(transform_map_to_base_link);
}


void PoseToTFPublisher::publishTFMapToOdom(const tf2::Transform& transform_map_to_base_link) {
	tf2::Transform transform_odom_to_base_link;
	if (!tf_collector_.lookForTransform(transform_odom_to_base_link, base_link_frame_id_, odom_frame_id_, ros::Time::now())) {
		return;
	}

	// map_base = map_odom * odom_base
	// map_odom = map_base * inverse(odom_base)
	tf2::Transform transform_map_to_odom = transform_map_to_base_link * transform_odom_to_base_link.inverse();
	laserscan_to_pointcloud::tf_rosmsg_eigen_conversions::transformTF2ToMsg(transform_map_to_odom, transform_stamped_map_to_base_link_.transform);

	publishTFMapToOdom();
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </TFPublisher-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// =============================================================================   <private-section>   =========================================================================
// =============================================================================   </private-section>  =========================================================================

} /* namespace dynamic_robot_localization */
