#pragma once

/**\file cluster_sorters.h
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// std includes
#include <memory>
#include <string>
#include <utility>
#include <vector>

// ROS includes
#include <tf2/LinearMath/Transform.h>

// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>

// project includes
#include <dynamic_robot_localization/common/configurable_object.h>
#include <dynamic_robot_localization/common/impl/pointcloud_utils.hpp>
#include <dynamic_robot_localization/common/math_utils.h>

// other includes
#include <laserscan_to_pointcloud/tf_collector.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {
// ############################################################################   cluster_sorters   ############################################################################

template <typename PointT>
class ClusterSorter : public ConfigurableObject {
	public:
		using Ptr = std::shared_ptr< ClusterSorter >;
		using ConstPtr = std::shared_ptr< const ClusterSorter >;

		ClusterSorter(bool ascending_sort) : ascending_sort_(ascending_sort), tf_collector_(nullptr) {}
		virtual ~ClusterSorter() {}

		virtual void setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace = "");
		virtual void loadTfNameForSortigClusters();
		virtual bool retrieveTfForSortigClusters(const std::string& cloud_frame_id, const ros::Time& cloud_time);
		virtual void sortClusters(const typename pcl::PointCloud<PointT>::Ptr& input_cloud, const std::vector<pcl::PointIndices>& cluster_indices,
				size_t sorted_clusters_subset_start_index, size_t sorted_clusters_subset_end_index, std::vector<size_t>& selected_clusters_out) = 0;
		void setClustersOrigins(pcl::PointCloud<pcl::PointXYZ>::Ptr& clusters_origins) { clusters_origins_ = clusters_origins; }
		pcl::PointCloud<pcl::PointXYZ>::Ptr& getClustersOrigins() { return clusters_origins_; }
		laserscan_to_pointcloud::TFCollector* getTfCollector() { return tf_collector_; }
		void setTfCollector(laserscan_to_pointcloud::TFCollector* tfCollector) { tf_collector_ = tfCollector; }

	protected:
		std::string configuration_namespace_;
		ros::NodeHandlePtr private_node_handle_;
		bool ascending_sort_;
		pcl::PointCloud<pcl::PointXYZ>::Ptr clusters_origins_; // if empty, centroids will be computed (if needed)
		laserscan_to_pointcloud::TFCollector* tf_collector_;
		ros::Duration tf_timeout_;
		std::string tf_name_for_sorting_clusters_;
		bool load_tf_name_for_sorting_clusters_before_processing_;
		bool use_cloud_time_for_retrieving_tf_transform_;
		tf2::Transform transform_from_cloud_frame_to_sorting_tf_frame_;
};


template <typename PointT>
class ClusterSizeSorter : public ClusterSorter<PointT> {
	public:
		ClusterSizeSorter(bool ascending_sort) : ClusterSorter<PointT>(ascending_sort) {}
		virtual ~ClusterSizeSorter() {}

		virtual void sortClusters(const typename pcl::PointCloud<PointT>::Ptr& input_cloud, const std::vector<pcl::PointIndices>& cluster_indices,
				size_t sorted_clusters_subset_start_index, size_t sorted_clusters_subset_end_index, std::vector<size_t>& selected_clusters_out);
};


template <typename PointT>
class DistanceToOriginSorter : public ClusterSorter<PointT> {
	public:
		DistanceToOriginSorter(bool ascending_sort) : ClusterSorter<PointT>(ascending_sort) {}
		virtual ~DistanceToOriginSorter() {}

		virtual void sortClusters(const typename pcl::PointCloud<PointT>::Ptr& input_cloud, const std::vector<pcl::PointIndices>& cluster_indices,
				size_t sorted_clusters_subset_start_index, size_t sorted_clusters_subset_end_index, std::vector<size_t>& selected_clusters_out);
		virtual float getDistanceSquared(const pcl::PointXYZ& point);
};


template <typename PointT>
class AxisValueSorter : public DistanceToOriginSorter<PointT> {
	public:
		enum Axis { X, Y, Z };

		AxisValueSorter(bool ascending_sort, Axis axis) : DistanceToOriginSorter<PointT>(ascending_sort), axis_(axis) {}
		virtual ~AxisValueSorter() {}

		float getDistanceSquared(const pcl::PointXYZ& point) override;

	protected:
		Axis axis_;
};


} /* namespace dynamic_robot_localization */
