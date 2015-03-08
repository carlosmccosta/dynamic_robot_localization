#pragma once

/**\file normal_estimator.h
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <macros>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </macros>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// std includes
#include <string>

// ROS includes
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <nav_msgs/OccupancyGrid.h>

// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>

// external libs includes
#include <boost/smart_ptr/shared_ptr.hpp>

// project includes
#include <dynamic_robot_localization/common/configurable_object.h>
#include <dynamic_robot_localization/curvature_estimators/curvature_estimator.h>

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {
// ###########################################################################   normal_estimator   ############################################################################
/**
 * \brief Description...
 */
template <typename PointT>
class NormalEstimator : public ConfigurableObject {
	// ========================================================================   <public-section>   ===========================================================================
	public:
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <typedefs>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		typedef boost::shared_ptr< NormalEstimator<PointT> > Ptr;
		typedef boost::shared_ptr< const NormalEstimator<PointT> > ConstPtr;
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </typedefs>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <enums>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </enums>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constants>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		#define VISUALIZER_NORMALS_ID "normals"
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constants>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		NormalEstimator();
		virtual ~NormalEstimator() {}
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <NormalEstimator-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		virtual void setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace = "");
		virtual void estimateNormals(typename pcl::PointCloud<PointT>::Ptr& pointcloud,
				typename pcl::PointCloud<PointT>::Ptr& surface,
				typename pcl::search::KdTree<PointT>::Ptr& surface_search_method,
				tf2::Transform& viewpoint_guess,
				typename pcl::PointCloud<PointT>::Ptr& pointcloud_with_normals_out);

		void displayNormals(typename pcl::PointCloud<PointT>::Ptr& pointcloud_with_normals);
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </NormalEstimator-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <gets>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		bool getDisplayNormals() const { return display_normals_; }
		bool getDisplayOccupancyGridPointcloud() const { return display_occupancy_grid_pointcloud_; }
		int getOccupancyGridAnalysisK() const { return occupancy_grid_analysis_k_; }
		double getOccupancyGridAnalysisRadius() const { return occupancy_grid_analysis_radius_; }
		double getOccupancyGridAnalysisRadiusResolutionPercentage() const { return occupancy_grid_analysis_radius_resolution_percentage_; }
		nav_msgs::OccupancyGridConstPtr getOccupancyGridMsg() { return occupancy_grid_msg_; }
		typename CurvatureEstimator<PointT>::Ptr getCurvatureEstimator() { return curvature_estimator_; }
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </gets>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <sets>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		void setOccupancyGridMsg(const nav_msgs::OccupancyGridConstPtr& occupancy_grid_msg) { occupancy_grid_msg_ = occupancy_grid_msg; }
		void resetOccupancyGridMsg() { occupancy_grid_msg_.reset(); }
		void setCurvatureEstimator(const typename CurvatureEstimator<PointT>::Ptr& curvature_estimator) { curvature_estimator_ = curvature_estimator; }
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </sets>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// ========================================================================   </public-section>  ===========================================================================

	// ========================================================================   <protected-section>   ========================================================================
	protected:
		bool display_normals_;
		bool display_occupancy_grid_pointcloud_;
		int occupancy_grid_analysis_k_;
		double occupancy_grid_analysis_radius_;
		double occupancy_grid_analysis_radius_resolution_percentage_;
		nav_msgs::OccupancyGridConstPtr occupancy_grid_msg_;
		typename CurvatureEstimator<PointT>::Ptr curvature_estimator_;
	// ========================================================================   </protected-section>  ========================================================================
};

} /* namespace dynamic_robot_localization */


#ifdef DRL_NO_PRECOMPILE
#include <dynamic_robot_localization/normal_estimators/impl/normal_estimator.hpp>
#endif

