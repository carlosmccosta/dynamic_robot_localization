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
#include <utility>
#include <vector>

// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/common/centroid.h>

// project includes
#include <dynamic_robot_localization/common/impl/pointcloud_utils.hpp>
#include <dynamic_robot_localization/common/math_utils.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {
// ############################################################################   cluster_sorters   ############################################################################

template <typename PointT>
class ClusterSorter {
	public:
		using Ptr = std::shared_ptr< ClusterSorter >;
		using ConstPtr = std::shared_ptr< const ClusterSorter >;

		ClusterSorter(bool ascending_sort) : ascending_sort_(ascending_sort) {}
		virtual ~ClusterSorter() {}

		virtual void sortClusters(const typename pcl::PointCloud<PointT>::Ptr& input_cloud, const std::vector<pcl::PointIndices>& cluster_indices,
				size_t sorted_clusters_subset_start_index, size_t sorted_clusters_subset_end_index, std::vector<size_t>& selected_clusters_out) = 0;
		void setClustersOrigins(pcl::PointCloud<pcl::PointXYZ>::Ptr& clusters_origins) { clusters_origins_ = clusters_origins; }
		pcl::PointCloud<pcl::PointXYZ>::Ptr& getClustersOrigins() { return clusters_origins_; }

	protected:
		bool ascending_sort_;
		pcl::PointCloud<pcl::PointXYZ>::Ptr clusters_origins_; // if empty, centroids will be computed (if needed)
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
		virtual float getDistance(const pcl::PointXYZ& point);
};


template <typename PointT>
class AxisValueSorter : public DistanceToOriginSorter<PointT> {
	public:
		enum Axis { X, Y, Z };

		AxisValueSorter(bool ascending_sort, Axis axis) : DistanceToOriginSorter<PointT>(ascending_sort), axis_(axis) {}
		virtual ~AxisValueSorter() {}

		virtual float getDistance(const pcl::PointXYZ& point);

	protected:
		Axis axis_;
};


} /* namespace dynamic_robot_localization */
