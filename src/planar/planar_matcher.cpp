/**\file planar_matcher.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include "dynamic_robot_localization/planar/planar_matcher.h"
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {
// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
PlanarMatcher::PlanarMatcher(double max_correspondence_distance, double transformation_epsilon, double euclidean_fitness_epsilon, int max_number_of_iterations) :
		reference_pointcloud_(new PointCloudT()),
		cloud_matcher_(new pcl::IterativeClosestPoint<PointT, PointT>()),
		threshold_for_map_cell_as_obstacle_(95) {

	cloud_matcher_->setMaxCorrespondenceDistance(max_correspondence_distance);
	cloud_matcher_->setTransformationEpsilon(transformation_epsilon);
	cloud_matcher_->setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
	cloud_matcher_->setMaximumIterations(max_number_of_iterations);

	reference_pointcloud_->height = 1;
	reference_pointcloud_->is_dense = false;
	reference_pointcloud_->header.stamp = ros::Time::now().toNSec() / 1e3;
}

PlanarMatcher::~PlanarMatcher() { }
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <PlanarMatcher-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
bool PlanarMatcher::createReferencePointcloudFromMap(const nav_msgs::OccupancyGridConstPtr& planar_map) {
	if (planar_map->data.size() > 0
			&& (planar_map->data.size() == (planar_map->info.width * planar_map->info.height))
			/*&& (planar_map->header.stamp.toNSec() / 1e3) > reference_pointcloud_->header.stamp*/) {
		float map_resolution = planar_map->info.resolution;
		int map_width = planar_map->info.width;
		int map_height = planar_map->info.height;

		float map_origin_x = planar_map->info.origin.position.x;
		float map_origin_y = planar_map->info.origin.position.y;


		PointCloudT::Ptr reference_pointcloud_from_map(new PointCloudT());
		reference_pointcloud_from_map->height = 1;
		reference_pointcloud_from_map->is_dense = false;
		reference_pointcloud_from_map->header.frame_id = planar_map->header.frame_id;
		reference_pointcloud_from_map->header.stamp = planar_map->header.stamp.toNSec() / 1e3;
		reference_pointcloud_from_map->points.clear();

		size_t data_position = 0;
		for (int y = 0; y < map_height; ++y) {
			float cell_y_position = (float)y * map_resolution + map_origin_y;
			for (int x = 0; x < map_width; ++x) {
				if (planar_map->data[data_position] > threshold_for_map_cell_as_obstacle_) {
					float cell_x_position = (float)x * map_resolution + map_origin_x;
					reference_pointcloud_from_map->points.push_back(PointT(cell_x_position, cell_y_position, 0.0));
				}

				++data_position;
			}
		}

		reference_pointcloud_from_map->width = reference_pointcloud_from_map->points.size();
		reference_pointcloud_ = reference_pointcloud_from_map; // switch smart pointer
		cloud_matcher_->setInputTarget(reference_pointcloud_);

		return true;
	}

	return false;
}


double PlanarMatcher::alignPlanarPointclouds(const PointCloudT::Ptr& environment_cloud, PointCloudT::Ptr& environment_cloud_aligned) {
	if (reference_pointcloud_->points.empty() || environment_cloud->points.empty()) {
		return -1;
	}

	cloud_matcher_->setInputSource(environment_cloud);
	cloud_matcher_->align(*environment_cloud_aligned);

	if (cloud_matcher_->hasConverged()) {
		return cloud_matcher_->getFitnessScore();
	} else {
		return -1;
	}
}


Eigen::Vector3f PlanarMatcher::correctPose(const Eigen::Vector3f& current_pose) {
	return (Eigen::Transform<float, 3, Eigen::Affine>(cloud_matcher_->getFinalTransformation())) * current_pose;
}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </PlanarMatcher-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// =============================================================================   <private-section>   =========================================================================
// =============================================================================   </private-section>  =========================================================================


} /* namespace dynamic_robot_localization */
