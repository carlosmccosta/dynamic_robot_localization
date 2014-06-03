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
		add_odometry_displacement_(false), map_received_(false), number_poses_published_(0), node_handle_(node_handle), private_node_handle_(private_node_handle) {

	// subscription topic names fields
	private_node_handle_->param("planar_pointcloud_topic", planar_pointcloud_topic_, std::string("planar_pointcloud"));
	private_node_handle_->param("costmap_topic", costmap_topic_, std::string("map"));

	// publish topic names
	private_node_handle_->param("reference_map_pointcloud_publish_topic", reference_map_pointcloud_publish_topic_, std::string("reference_map_pointcloud"));
	private_node_handle_->param("aligned_pointcloud_publish_topic", aligned_pointcloud_publish_topic_, std::string("aligned_pointcloud"));
	private_node_handle_->param("pose_publish_topic", pose_publish_topic_, std::string("initialpose"));

	// configuration fields
	private_node_handle_->param("publish_tf_map_odom", publish_tf_map_odom_, true);
	private_node_handle_->param("base_link_frame_id", base_link_frame_id_, std::string("base_link"));
	double max_seconds_scan_age;
	private_node_handle_->param("max_seconds_scan_age", max_seconds_scan_age, 5.0);
	max_seconds_scan_age_.fromSec(max_seconds_scan_age);
	double min_seconds_between_scan_registration;
	private_node_handle_->param("min_seconds_between_scan_registration", min_seconds_between_scan_registration, 0.05);
	min_seconds_between_scan_registration_.fromSec(min_seconds_between_scan_registration);
	double min_seconds_between_map_update;
	private_node_handle_->param("min_seconds_between_map_update", min_seconds_between_map_update, 5.0);
	min_seconds_between_map_update_.fromSec(min_seconds_between_map_update);


	// icp configuration fields
	private_node_handle_->param("max_alignment_fitness", max_alignment_fitness_, 0.5);
	private_node_handle_->param("max_transformation_angle", max_transformation_angle_, 0.79);
	private_node_handle_->param("max_transformation_distance", max_transformation_distance_, 0.5);
	private_node_handle_->param("max_correspondence_distance", max_correspondence_distance_, 2.0);
	private_node_handle_->param("transformation_epsilon", transformation_epsilon_, 1e-6);
	private_node_handle_->param("euclidean_fitness_epsilon", euclidean_fitness_epsilon_, 0.001);
	private_node_handle_->param("max_number_of_iterations", max_number_of_iterations_, 1000);
	planar_matcher_ = PlanarMatcher(max_correspondence_distance_, transformation_epsilon_, euclidean_fitness_epsilon_, max_number_of_iterations_);


	if (publish_tf_map_odom_) {
		pose_to_tf_publisher_.readConfigurationFromParameterServer(node_handle, private_node_handle);
		pose_to_tf_publisher_.publishInitialPoseFromParameterServer();
	}

	dynamic_reconfigure::Server<dynamic_robot_localization::PlanarLocalizationConfig>::CallbackType callback_dynamic_reconfigure =
				boost::bind(&dynamic_robot_localization::PlanarLocalization::dynamicReconfigureCallback, this, _1, _2);
		dynamic_reconfigure_server_.setCallback(callback_dynamic_reconfigure);

}

PlanarLocalization::~PlanarLocalization() {
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <PlanarLocalization-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void dynamic_robot_localization::PlanarLocalization::startLocalization() {
	laserscan_cloud_subscriber_ = node_handle_->subscribe(planar_pointcloud_topic_, 100, &dynamic_robot_localization::PlanarLocalization::processLaserScanCloud, this);
	costmap_subscriber_ = node_handle_->subscribe(costmap_topic_, 5, &dynamic_robot_localization::PlanarLocalization::processCostmap, this);
	map_pointcloud_publisher_ = node_handle_->advertise<sensor_msgs::PointCloud2>(reference_map_pointcloud_publish_topic_, 10);
	aligned_pointcloud_publisher_ = node_handle_->advertise<sensor_msgs::PointCloud2>(aligned_pointcloud_publish_topic_, 10);
	pose_publisher_ = node_handle_->advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_publish_topic_, 10);

	if (publish_tf_map_odom_) {
		ros::Rate publish_rate(pose_to_tf_publisher_.getPublishRate());
		while (ros::ok()) {
			pose_to_tf_publisher_.publishTFMapToOdom();
			publish_rate.sleep();
			ros::spinOnce();
		}
	} else {
		ros::spin();
	}
}


void PlanarLocalization::stopLocalization() {
	laserscan_cloud_subscriber_.shutdown();
	costmap_subscriber_.shutdown();
	map_pointcloud_publisher_.shutdown();
	aligned_pointcloud_publisher_.shutdown();
	pose_publisher_.shutdown();
}


void PlanarLocalization::processCostmap(const nav_msgs::OccupancyGridConstPtr& planar_map) {
	if (!map_received_ || (ros::Time::now() - last_map_received_time_) > min_seconds_between_map_update_) {
		last_map_received_time_ = ros::Time::now();
		if (planar_matcher_.createReferencePointcloudFromMap(planar_map)) {
			map_received_ = true;
		}

		if (!reference_map_pointcloud_publish_topic_.empty() /*&& map_pointcloud_publisher_.getNumSubscribers() > 0*/) {
			sensor_msgs::PointCloud2Ptr reference_pointcloud(new sensor_msgs::PointCloud2());
			pcl::toROSMsg(*(planar_matcher_.getReferencePointcloud()), *reference_pointcloud);
			map_pointcloud_publisher_.publish(reference_pointcloud);
		}
	}
}


void PlanarLocalization::processLaserScanCloud(const sensor_msgs::PointCloud2ConstPtr& laserscan_cloud) {
	ros::Duration scan_age = ros::Time::now() - last_matched_scan_time_;

	if (map_received_ && scan_age > min_seconds_between_scan_registration_ && scan_age < max_seconds_scan_age_) {
		last_matched_scan_time_ = laserscan_cloud->header.stamp;

		tf2::Transform pose_tf;
		if (!pose_to_tf_publisher_.getTfCollector().lookForTransform(pose_tf, laserscan_cloud->header.frame_id, base_link_frame_id_, laserscan_cloud->header.stamp)) {
			ROS_DEBUG_STREAM("Dropping scan because tf between " << laserscan_cloud->header.frame_id << " and " << base_link_frame_id_ << " isn't available");
			return;
		}

		PlanarMatcher::PointCloudT::Ptr pointcloud(new PlanarMatcher::PointCloudT());
		PlanarMatcher::PointCloudT::Ptr aligned_pointcloud(new PlanarMatcher::PointCloudT());
		pcl::fromROSMsg(*laserscan_cloud, *pointcloud);
		resetPointCloudHeight(pointcloud);

		double alignmentFitness = planar_matcher_.alignPlanarPointclouds(pointcloud, aligned_pointcloud); // alignmentFitness < 0 if there was no alignment
		if (alignmentFitness >= 0 && alignmentFitness < max_alignment_fitness_) {
			ROS_DEBUG_STREAM("Registered cloud with " << alignmentFitness << " alignment fitness");

			geometry_msgs::PoseWithCovarianceStampedPtr pose(new geometry_msgs::PoseWithCovarianceStamped());
			pose->header.frame_id = laserscan_cloud->header.frame_id;
			pose->header.seq = number_poses_published_++;

			if (add_odometry_displacement_) {
				pose->header.stamp = ros::Time::now();
			} else {
				pose->header.stamp = laserscan_cloud->header.stamp;
			}


			tf2::Transform pose_correction;
			laserscan_to_pointcloud::tf_rosmsg_eigen_conversions::transformMatrixToTF2(planar_matcher_.getCloudMatcher()->getFinalTransformation(), pose_correction);

			double transform_distance = pose_correction.getOrigin().length();
			double transform_angle = pose_correction.getRotation().getAngle();
			if (transform_distance > max_transformation_distance_ || transform_angle > max_transformation_angle_) {
				ROS_DEBUG_STREAM("Dropping scan because pose correction exceeded bounds (translation: " << transform_distance << " | rotation: " << transform_angle);
				return;
			}

			tf2::Transform pose_corrected = pose_correction * pose_tf;

			if (add_odometry_displacement_)
				pose_to_tf_publisher_.addOdometryDisplacementToTransform(pose_corrected, laserscan_cloud->header.stamp);

			if (publish_tf_map_odom_)
				pose_to_tf_publisher_.publishTFMapToOdom(pose_corrected);

			laserscan_to_pointcloud::tf_rosmsg_eigen_conversions::transformTF2ToMsg(pose_corrected, pose->pose.pose);
//			todo: fill covariance
//			pose->pose.covariance

			pose_publisher_.publish(pose);

			if (!aligned_pointcloud_publish_topic_.empty() && aligned_pointcloud_publisher_.getNumSubscribers() > 0) {
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


void PlanarLocalization::dynamicReconfigureCallback(dynamic_robot_localization::PlanarLocalizationConfig& config, uint32_t level) {
	if (level == 1) {
			ROS_INFO_STREAM("LaserScanToPointcloudAssembler dynamic reconfigure (level=" << level << ") -> " \
					<< "\n\t[planar_pointcloud_topic]: " 					<< planar_pointcloud_topic_ 											<< " -> " << config.planar_pointcloud_topic \
					<< "\n\t[costmap_topic]: " 								<< costmap_topic_														<< " -> " << config.costmap_topic \
					<< "\n\t[reference_map_pointcloud_publish_topic]: "		<< reference_map_pointcloud_publish_topic_ 								<< " -> " << config.reference_map_pointcloud_publish_topic \
					<< "\n\t[aligned_pointcloud_publish_topic]: "			<< aligned_pointcloud_publish_topic_ 									<< " -> " << config.aligned_pointcloud_publish_topic \
					<< "\n\t[pose_publish_topic]: " 						<< pose_publish_topic_ 													<< " -> " << config.pose_publish_topic \
					<< "\n\t[publish_tf_map_odom]: "					 	<< publish_tf_map_odom_				 									<< " -> " << config.publish_tf_map_odom \
					<< "\n\t[base_link_frame_id]: "					 		<< base_link_frame_id_				 									<< " -> " << config.base_link_frame_id \
					<< "\n\t[max_seconds_scan_age]: " 						<< max_seconds_scan_age_.toSec()										<< " -> " << config.max_seconds_scan_age \
					<< "\n\t[min_seconds_between_laserscan_registration]: " << min_seconds_between_scan_registration_.toSec()						<< " -> " << config.min_seconds_between_laserscan_registration \
					<< "\n\t[min_seconds_between_map_update]: " 			<< min_seconds_between_map_update_.toSec() 								<< " -> " << config.min_seconds_between_map_update \
					<< "\n\t[max_alignment_fitness]: " 						<< max_alignment_fitness_					 							<< " -> " << config.max_alignment_fitness \
					<< "\n\t[max_transformation_angle]: " 					<< max_transformation_angle_				 							<< " -> " << config.max_transformation_angle \
					<< "\n\t[max_transformation_distance]: " 				<< max_transformation_distance_					 						<< " -> " << config.max_transformation_distance \
					<< "\n\t[max_correspondence_distance]: " 				<< planar_matcher_.getCloudMatcher()->getMaxCorrespondenceDistance() 	<< " -> " << config.max_correspondence_distance \
					<< "\n\t[transformation_epsilon]: " 					<< planar_matcher_.getCloudMatcher()->getTransformationEpsilon() 		<< " -> " << config.transformation_epsilon \
					<< "\n\t[euclidean_fitness_epsilon]: " 					<< planar_matcher_.getCloudMatcher()->getEuclideanFitnessEpsilon() 		<< " -> " << config.euclidean_fitness_epsilon \
					<< "\n\t[max_number_of_iterations]: " 					<< planar_matcher_.getCloudMatcher()->getMaximumIterations()			<< " -> " << config.max_number_of_iterations);

			// Subscribe topics
			if (!config.planar_pointcloud_topic.empty() && planar_pointcloud_topic_ != config.planar_pointcloud_topic) {
				planar_pointcloud_topic_ = config.planar_pointcloud_topic;
				laserscan_cloud_subscriber_.shutdown();
				laserscan_cloud_subscriber_ = node_handle_->subscribe(planar_pointcloud_topic_, 100, &dynamic_robot_localization::PlanarLocalization::processLaserScanCloud, this);
			}

			if (!config.costmap_topic.empty() && costmap_topic_ != config.costmap_topic) {
				costmap_topic_ = config.costmap_topic;
				costmap_subscriber_.shutdown();
				costmap_subscriber_ = node_handle_->subscribe(costmap_topic_, 5, &dynamic_robot_localization::PlanarLocalization::processCostmap, this);
			}

			// Publish topics
			if (!config.reference_map_pointcloud_publish_topic.empty() && reference_map_pointcloud_publish_topic_ != config.reference_map_pointcloud_publish_topic) {
				reference_map_pointcloud_publish_topic_ = config.reference_map_pointcloud_publish_topic;
				map_pointcloud_publisher_.shutdown();
				map_pointcloud_publisher_ = node_handle_->advertise<sensor_msgs::PointCloud2>(reference_map_pointcloud_publish_topic_, 10);
			}

			if (!config.aligned_pointcloud_publish_topic.empty() && aligned_pointcloud_publish_topic_ != config.aligned_pointcloud_publish_topic) {
				aligned_pointcloud_publish_topic_ = config.aligned_pointcloud_publish_topic;
				aligned_pointcloud_publisher_.shutdown();
				aligned_pointcloud_publisher_ = node_handle_->advertise<sensor_msgs::PointCloud2>(aligned_pointcloud_publish_topic_, 10);
			}

			if (!config.pose_publish_topic.empty() && pose_publish_topic_ != config.pose_publish_topic) {
				pose_publish_topic_ = config.pose_publish_topic;
				pose_publisher_.shutdown();
				pose_publisher_ = node_handle_->advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_publish_topic_, 10);
			}

			// Configurations
			publish_tf_map_odom_ = config.publish_tf_map_odom;
			base_link_frame_id_ = config.base_link_frame_id;
			max_seconds_scan_age_.fromSec(config.max_seconds_scan_age);
			min_seconds_between_scan_registration_.fromSec(config.min_seconds_between_laserscan_registration);
			min_seconds_between_map_update_.fromSec(config.min_seconds_between_map_update);

			// ICP configuration
			max_alignment_fitness_ = config.max_alignment_fitness;
			max_transformation_angle_ = config.max_transformation_angle;
			max_transformation_distance_ = config.max_transformation_distance;
			planar_matcher_.getCloudMatcher()->setMaxCorrespondenceDistance(config.max_correspondence_distance);
			planar_matcher_.getCloudMatcher()->setTransformationEpsilon(config.transformation_epsilon);
			planar_matcher_.getCloudMatcher()->setEuclideanFitnessEpsilon(config.euclidean_fitness_epsilon);
			planar_matcher_.getCloudMatcher()->setMaximumIterations(config.max_number_of_iterations);
		}
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </PlanarLocalization-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// =============================================================================   <private-section>   =========================================================================
// =============================================================================   </private-section>  =========================================================================

} /* namespace dynamic_robot_localization */

