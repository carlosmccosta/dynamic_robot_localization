/**\file planar_localization.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include "dynamic_robot_localization/planar/planar_localization.h"
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
PlanarLocalization::PlanarLocalization(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle) :
		map_received_(false), number_tfs_published_(0), map_frame_id_("map"),
		node_handle_(node_handle), private_node_handle_(private_node_handle) {

	// subscription topic names fields
	private_node_handle_->param("planar_pointcloud_topic", planar_pointcloud_topic_, std::string("planar_pointcloud"));
	private_node_handle_->param("costmap_topic", costmap_topic_, std::string("map"));
	private_node_handle_->param("initial_pose_topic", initial_pose_topic_, std::string("initialpose"));

	// publish topic names
	private_node_handle_->param("reference_map_pointcloud_publish_topic", reference_map_pointcloud_publish_topic_, std::string("reference_map_pointcloud"));
	private_node_handle_->param("aligned_pointcloud_publish_topic", aligned_pointcloud_publish_topic_, std::string("aligned_pointcloud"));
	private_node_handle_->param("pose_publish_topic", pose_publish_topic_, std::string("pose_corrected"));

	// control fields
	double min_seconds_between_laserscan_registration;
	private_node_handle_->param("min_seconds_between_laserscan_registration", min_seconds_between_laserscan_registration, 0.5);
	min_seconds_between_laserscan_registration_.fromSec(min_seconds_between_laserscan_registration);
	double min_seconds_between_map_update;
	private_node_handle_->param("min_seconds_between_map_update", min_seconds_between_map_update, 5.0);
	min_seconds_between_map_update_.fromSec(min_seconds_between_map_update);
	private_node_handle_->param("publish_tf_map_to_odom", publish_tf_map_to_odom_, true);
	private_node_handle_->param("odom_frame_id", odom_frame_id_, std::string("odom"));
	private_node_handle_->param("base_link_frame_id", base_link_frame_id_, std::string("base_link"));

	// icp configuration fields
	private_node_handle_->param("max_alignment_fitness_", max_alignment_fitness_, 0.5);
	private_node_handle_->param("max_correspondence_distance", max_correspondence_distance_, 0.1);
	private_node_handle_->param("transformation_epsilon", transformation_epsilon_, 1e-6);
	private_node_handle_->param("euclidean_fitness_epsilon", euclidean_fitness_epsilon_, 1.0);
	private_node_handle_->param("max_number_of_iterations_", max_number_of_iterations_, 250);
	planar_matcher_ = PlanarMatcher(max_correspondence_distance_, transformation_epsilon_, euclidean_fitness_epsilon_, max_number_of_iterations_);

	// initial pose tf
	double yaw, x, y, z;
	private_node_handle_->param("initial_yaw", yaw, 0.0);
	private_node_handle_->param("initial_x", x, 0.0);
	private_node_handle_->param("initial_y", y, 0.0);
	private_node_handle_->param("initial_z", z, 0.0);
	publishTFMapToOdomFromPose(yaw, x, y, z);
}

PlanarLocalization::~PlanarLocalization() { }
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <PlanarLocalization-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void dynamic_robot_localization::PlanarLocalization::startLocalization() {
	laserscan_cloud_subscriber_ = node_handle_->subscribe(planar_pointcloud_topic_, 100, &dynamic_robot_localization::PlanarLocalization::processLaserScanCloud, this);
	costmap_subscriber_ = node_handle_->subscribe(costmap_topic_, 5, &dynamic_robot_localization::PlanarLocalization::processCostmap, this);
	initial_pose_subscriber_ = node_handle_->subscribe(initial_pose_topic_, 5, &dynamic_robot_localization::PlanarLocalization::publishTFMapToOdomFromPose, this);
	map_pointcloud_publisher_ = node_handle_->advertise<sensor_msgs::PointCloud2>(reference_map_pointcloud_publish_topic_, 10);
	aligned_pointcloud_publisher_ = node_handle_->advertise<sensor_msgs::PointCloud2>(aligned_pointcloud_publish_topic_, 10);
	pose_publisher_ = node_handle_->advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_publish_topic_, 10);
}


void PlanarLocalization::stopLocalization() {
	laserscan_cloud_subscriber_.shutdown();
	costmap_subscriber_.shutdown();
	initial_pose_subscriber_.shutdown();
	map_pointcloud_publisher_.shutdown();
	aligned_pointcloud_publisher_.shutdown();
	pose_publisher_.shutdown();
}


void PlanarLocalization::processCostmap(const nav_msgs::OccupancyGridConstPtr& planar_map) {
	if (!map_received_ || (ros::Time::now() - last_map_received_time_) > min_seconds_between_map_update_) {
		last_map_received_time_ = ros::Time::now();
		if (planar_matcher_.createReferencePointcloudFromMap(planar_map)) {
			map_received_ = true;
			map_frame_id_ = planar_map->header.frame_id;
		}

		if (!reference_map_pointcloud_publish_topic_.empty()) {
			sensor_msgs::PointCloud2Ptr reference_pointcloud(new sensor_msgs::PointCloud2());
			pcl::toROSMsg(*(planar_matcher_.getReferencePointcloud()), *reference_pointcloud);
			map_pointcloud_publisher_.publish(reference_pointcloud);
		}
	}
}


void PlanarLocalization::processLaserScanCloud(const sensor_msgs::PointCloud2ConstPtr& laserscan_cloud) {
	if (map_received_ && (ros::Time::now() - last_matched_scan_time_) > min_seconds_between_laserscan_registration_) {
		last_matched_scan_time_ = ros::Time::now();
		PlanarMatcher::PointCloudT::Ptr pointcloud(new PlanarMatcher::PointCloudT());
		PlanarMatcher::PointCloudT::Ptr aligned_pointcloud(new PlanarMatcher::PointCloudT());
		pcl::fromROSMsg(*laserscan_cloud, *pointcloud);
		resetPointCloudHeight(pointcloud);

		double alignmentFitness = planar_matcher_.alignPlanarPointclouds(pointcloud, aligned_pointcloud); // alignmentFitness < 0 if there was no alignment
		if (alignmentFitness >= 0  && alignmentFitness < max_alignment_fitness_) {
			publishPoseCorrectedFromCurrentCloudRegistration();
			if (publish_tf_map_to_odom_) {
//				publishTFMapToOdomFromCurrentCloudRegistration();
				publishTFMapToOdomFromPose(1.5, -3.2, -3.75, 0.0);
			}

			if (!aligned_pointcloud_publish_topic_.empty()) {
				sensor_msgs::PointCloud2Ptr aligned_pointcloud_msg(new sensor_msgs::PointCloud2());
				pcl::toROSMsg(*aligned_pointcloud, *aligned_pointcloud_msg);
				aligned_pointcloud_publisher_.publish(aligned_pointcloud_msg);
			}
		}
	}
}


void PlanarLocalization::resetPointCloudHeight(PlanarMatcher::PointCloudT::Ptr& pointcloud, float height) {
	size_t number_of_points = pointcloud->points.size();
	for (size_t i = 0; i < number_of_points; ++i) {
		pointcloud->points[i].z = height;
	}
}

void PlanarLocalization::publishPoseCorrectedFromCurrentCloudRegistration() {

}


void PlanarLocalization::publishTFMapToOdomFromCurrentCloudRegistration() {
	tf2::Transform transform_map_to_base_link;
	laserscan_to_pointcloud::tf_rosmsg_eigen_conversions::transformMatrixToTF2(planar_matcher_.getCloudMatcher()->getFinalTransformation(), transform_map_to_base_link);
	publishTFMapToOdom(transform_map_to_base_link);
}


void PlanarLocalization::publishTFMapToOdom(const tf2::Transform& transform_map_to_base_link) {
	tf2::Transform transform_odom_to_base_link;
	if (!tf_collector_.lookForTransform(transform_odom_to_base_link, base_link_frame_id_, odom_frame_id_, ros::Time::now())) {
		return;
	}

	// map_base = map_odom * odom_base
	// map_odom = map_base * inverse(odom_base)
	tf2::Transform transform_map_to_odom = transform_map_to_base_link * transform_odom_to_base_link.inverse();

	geometry_msgs::TransformStamped transformStamped;
	transformStamped.child_frame_id = odom_frame_id_;
	transformStamped.header.frame_id = map_frame_id_;
	transformStamped.header.seq = number_tfs_published_++;
	transformStamped.header.stamp = ros::Time::now();
	laserscan_to_pointcloud::tf_rosmsg_eigen_conversions::transformTF2ToMsg(transform_map_to_odom, transformStamped.transform);

	transform_broadcaster_.sendTransform(transformStamped);
}


void dynamic_robot_localization::PlanarLocalization::publishTFMapToOdomFromPose(double yaw, double x, double y, double z) {
	tf2::Quaternion orientation;
	orientation.setRPY(0.0, 0.0, yaw);

	publishTFMapToOdom(tf2::Transform(orientation, tf2::Vector3(x, y, z)));
}


void dynamic_robot_localization::PlanarLocalization::publishTFMapToOdomFromPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose) {
	tf2::Transform transform_map_to_base_link(
			tf2::Quaternion(pose->pose.pose.orientation.x, pose->pose.pose.orientation.y, pose->pose.pose.orientation.z, pose->pose.pose.orientation.w),
			tf2::Vector3(pose->pose.pose.position.x, pose->pose.pose.position.y, pose->pose.pose.position.z));

	publishTFMapToOdom(transform_map_to_base_link);
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </PlanarLocalization-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// =============================================================================   <private-section>   =========================================================================
// =============================================================================   </private-section>  =========================================================================

} /* namespace dynamic_robot_localization */
