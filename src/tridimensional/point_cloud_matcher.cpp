/**\file point_cloud_matcher.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include "dynamic_robot_localization/tridimensional/point_cloud_matcher.h"
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
PointCloudMatcher::PointCloudMatcher(double transformation_epsilon, double max_correspondence_distance, int max_number_of_iterations) :
			reference_pointcloud_(new pcl::PointCloud<pcl::PointXYZ>()),
			cloud_matcher_(new pcl::IterativeClosestPoint()) {
	cloud_matcher_->setTransformationEpsilon(transformation_epsilon);
	cloud_matcher_->setMaxCorrespondenceDistance(max_correspondence_distance);
	cloud_matcher_->setMaximumIterations(max_number_of_iterations);

}

PointCloudMatcher::~PointCloudMatcher() { }
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <PointCloudMatcher-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
bool dynamic_robot_localization::PointCloudMatcher::loadReferencePointCloud(const std::string& refrence_cloud_pcd_file_) {
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(refrence_cloud_pcd_file_, *reference_pointcloud_) == 0) {
		cloud_matcher_->setInputSource(reference_pointcloud_);
		return true;
	} else {
		return false;
	}
}


double dynamic_robot_localization::PointCloudMatcher::alignPointclouds(const PointCloudT::Ptr& target_cloud, PointCloudT::Ptr& target_cloud_aligned) {
	cloud_matcher_->setInputTarget(target_cloud);
	cloud_matcher_->align(*target_cloud_aligned);

	if (cloud_matcher_->hasConverged())
		return cloud_matcher_->getFitnessScore();
	else
		return -1;
}


Eigen::Vector3f dynamic_robot_localization::PointCloudMatcher::correctPose(const Eigen::Vector3f& current_pose, const pcl::Registration::Matrix4& final_transform) {
	return (Eigen::Transform<float, 3, Eigen::Affine>(final_transform)) * current_pose;
}


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </PointCloudMatcher-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// =============================================================================   <private-section>   =========================================================================
// =============================================================================   </private-section>  =========================================================================

} /* namespace dynamic_robot_localization */
