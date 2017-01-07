#pragma once

/**\file circular_pointcloud.h
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <macros>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </macros>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// std includes
#include <iterator>
#include <utility>
#include <vector>

// ROS includes

// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// external libs includes
#include <boost/smart_ptr/shared_ptr.hpp>

// project includes

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {
// ########################################################################   CircularBufferPointCloud   #######################################################################
/**
 * \brief Description...
 */
template <typename PointT>
class CircularBufferPointCloud {
	// ========================================================================   <public-section>   ===========================================================================
	public:
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <typedefs>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		typedef boost::shared_ptr< CircularBufferPointCloud<PointT> > Ptr;
		typedef boost::shared_ptr< const CircularBufferPointCloud<PointT> > ConstPtr;
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </typedefs>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <enums>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </enums>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constants>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constants>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		explicit CircularBufferPointCloud(size_t max_buffer_size = 1024, typename pcl::PointCloud<PointT>::Ptr pointcloud = typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>()));
		virtual ~CircularBufferPointCloud() {}
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <CircularBufferPointCloud-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		void insert(const PointT& new_element);
		void insert(const pcl::PointCloud<PointT>& new_elements);
		void insert(typename pcl::PointCloud<PointT>::const_iterator first, typename pcl::PointCloud<PointT>::const_iterator last);
		bool insertWithSwappping(pcl::PointCloud<PointT>& new_elements);
		bool insertWithSwappping(typename pcl::PointCloud<PointT>::iterator first, typename pcl::PointCloud<PointT>::iterator last);
		void insertReverse(const PointT& new_element);
		void insertReverse(const pcl::PointCloud<PointT>& new_elements);
		void insertReverse(typename pcl::PointCloud<PointT>::const_iterator first, typename pcl::PointCloud<PointT>::const_iterator last);
		void eraseNewest(size_t count = 1);
		void eraseOldest(size_t count = 1);

		typename pcl::PointCloud<PointT>::iterator begin() { return pointcloud_->begin(); }
		typename pcl::PointCloud<PointT>::const_iterator begin() const { return pointcloud_->begin(); }
		typename pcl::PointCloud<PointT>::iterator end() { return pointcloud_->end(); }
		typename pcl::PointCloud<PointT>::const_iterator end() const { return pointcloud_->end(); }
		PointT& operator[](size_t element_index) { return (*pointcloud_)[element_index]; }
		const PointT& operator[](size_t element_index) const { return (*pointcloud_)[element_index]; }
		PointT& at(size_t element_index) { return pointcloud_->at(element_index); }
		const PointT& at(size_t element_index) const { return pointcloud_->at(element_index); }
		PointT& front(size_t element_index) { return pointcloud_->front(); }
		const PointT& front(size_t element_index) const  { return pointcloud_->front(); }
		PointT& back(size_t element_index) { return pointcloud_->back(); }
		const PointT& back(size_t element_index) const { return pointcloud_->back(); }

		bool empty()  { return pointcloud_->empty(); }
		size_t size() { return pointcloud_->size(); }
		void resize(size_t number_elements) { pointcloud_->resize(number_elements); max_buffer_size_ = number_elements; }
		void reserve(size_t number_elements) { if (pointcloud_->size() < number_elements) { pointcloud_->reserve(number_elements); max_buffer_size_ = number_elements; } }
		void clear() { pointcloud_->clear(); }
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </CircularBufferPointCloud-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <gets>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		typename pcl::PointCloud<PointT>& getPointCloud() { return *pointcloud_; }
		typename pcl::PointCloud<PointT>::Ptr getPointCloudPtr() { return pointcloud_; }
		size_t getMaxBufferSize() const { return max_buffer_size_; }
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </gets>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <sets>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		void setMaxBufferSize(size_t max_buffer_size) { max_buffer_size_ = max_buffer_size; }
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </sets>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// ========================================================================   </public-section>  ===========================================================================

	// ========================================================================   <protected-section>   ========================================================================
	protected:
		size_t fillBuffer(size_t number_elements_to_insert, typename pcl::PointCloud<PointT>::const_iterator first);
		typename pcl::PointCloud<PointT>::Ptr pointcloud_;
		size_t next_insert_position_;
		size_t max_buffer_size_;
	// ========================================================================   </protected-section>  ========================================================================
};

} /* namespace dynamic_robot_localization */


#ifdef DRL_NO_PRECOMPILE
#include <dynamic_robot_localization/commom/impl/circular_buffer_pointcloud.hpp>
#endif

