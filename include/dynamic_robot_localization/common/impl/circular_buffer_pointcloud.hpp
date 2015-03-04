/**\file circular_pointcloud.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/common/circular_buffer_pointcloud.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
CircularBufferPointCloud<PointT>::CircularBufferPointCloud(size_t max_buffer_size, typename pcl::PointCloud<PointT>::Ptr pointcloud) : pointcloud_(pointcloud), max_buffer_size_(max_buffer_size) {
	if (pointcloud_->size() > max_buffer_size) {
		pointcloud_->resize(max_buffer_size_);
	}

	if (pointcloud_->size() == max_buffer_size) {
		next_insert_position_ = 0;
	} else {
		next_insert_position_ = pointcloud->size();
	}
}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <CircularBufferPointCloud-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT>
size_t CircularBufferPointCloud<PointT>::fillBuffer(size_t number_elements_to_insert, typename pcl::PointCloud<PointT>::const_iterator first) {
	if (number_elements_to_insert == 0) { return 0; }

	if (pointcloud_->size() < max_buffer_size_) { // filling the buffer
		size_t remaining_empty_slots_buffer = max_buffer_size_ - pointcloud_->size();
		size_t number_elements_inserted = std::min(number_elements_to_insert, remaining_empty_slots_buffer);

		if (next_insert_position_ == pointcloud_->size()) { 									// filling the buffer at the end
			pointcloud_->insert(pointcloud_->end(), first, first + number_elements_inserted);
		} else {																				// inserting elements (happens if vector size was modified)
			pointcloud_->insert(pointcloud_->begin() + next_insert_position_, first, first + number_elements_inserted);
		}

		next_insert_position_ += number_elements_inserted;

		if (number_elements_inserted < number_elements_to_insert) {
			number_elements_to_insert -= number_elements_inserted;
		} else {
			return 0;
		}
	}

	return number_elements_to_insert; // remaining elements to insert
}


template<typename PointT>
void CircularBufferPointCloud<PointT>::insert(const PointT& new_element) {
	if (pointcloud_->size() < max_buffer_size_) {
		if (next_insert_position_ == pointcloud_->size()) {
			pointcloud_->push_back(new_element); 												// filling the buffer at the end
		} else {
			pointcloud_->insert(pointcloud_->begin() + next_insert_position_, new_element); 	// inserting elements (happens if vector size was modified)
		}
	} else {
		(*pointcloud_)[next_insert_position_] = new_element; 									// replacing old elements
	}

	if (++next_insert_position_ >= max_buffer_size_) {
		next_insert_position_ = 0;																// wrap around
	}
}


template<typename PointT>
void CircularBufferPointCloud<PointT>::insert(const pcl::PointCloud<PointT>& new_elements) {
	if (new_elements.size() < max_buffer_size_) {
		insert(new_elements.begin(), new_elements.end());
	} else {
		insert(new_elements.begin(), new_elements.begin() + max_buffer_size_);
	}
}


template<typename PointT>
void CircularBufferPointCloud<PointT>::insert(typename pcl::PointCloud<PointT>::const_iterator first, typename pcl::PointCloud<PointT>::const_iterator last) {
	size_t number_elements_to_insert = std::distance(first, last);
	if (number_elements_to_insert > max_buffer_size_) {
		last = first + max_buffer_size_;
	}

	size_t remaining_number_elements_to_insert = fillBuffer(number_elements_to_insert, first);
	std::advance(first, number_elements_to_insert - remaining_number_elements_to_insert);

	if (remaining_number_elements_to_insert > 0) {
		if (next_insert_position_ + remaining_number_elements_to_insert < pointcloud_->size()) { // replacing old elements
			for (size_t i = 0; i < remaining_number_elements_to_insert; ++i) {
				(*pointcloud_)[next_insert_position_++] = *first++;
			}
		} else { // replacing old elements with wrap around
			size_t next_element_to_insert = 0;
			size_t number_elements_until_end_of_buffer = pointcloud_->size() - next_insert_position_;
			for (; next_element_to_insert < number_elements_until_end_of_buffer; ++next_element_to_insert) {
				(*pointcloud_)[next_insert_position_++] = *first++;
			}

			next_insert_position_ = 0;
			for (; next_element_to_insert < remaining_number_elements_to_insert; ++next_element_to_insert) {
				(*pointcloud_)[next_insert_position_++] = *first++;
			}
		}
	}
}


template<typename PointT>
bool CircularBufferPointCloud<PointT>::insertWithSwappping(pcl::PointCloud<PointT>& new_elements) {
	if (new_elements.size() < max_buffer_size_) {
		return insertWithSwappping(new_elements.begin(), new_elements.end());
	} else {
		return insertWithSwappping(new_elements.begin(), new_elements.begin() + max_buffer_size_);
	}
}


template<typename PointT>
bool CircularBufferPointCloud<PointT>::insertWithSwappping(typename pcl::PointCloud<PointT>::iterator first, typename pcl::PointCloud<PointT>::iterator last) {
	size_t number_elements_to_insert = std::distance(first, last);
	if (number_elements_to_insert > max_buffer_size_) {
		last = first + max_buffer_size_;
	}
	size_t remaining_number_elements_to_insert = fillBuffer(number_elements_to_insert, first);
	std::advance(first, number_elements_to_insert - remaining_number_elements_to_insert);

	if (next_insert_position_ + remaining_number_elements_to_insert < pointcloud_->size()) { // swapping old elements
		for (size_t i = 0; i < remaining_number_elements_to_insert; ++i) {
			std::swap((*pointcloud_)[next_insert_position_++], *first++);
		}
	} else { // swapping old elements with wrap around
		size_t next_element_to_insert = 0;
		size_t number_elements_until_end_of_buffer = pointcloud_->size() - next_insert_position_;
		for (; next_element_to_insert < number_elements_until_end_of_buffer; ++next_element_to_insert) {
			std::swap((*pointcloud_)[next_insert_position_++], *first++);
		}

		next_insert_position_ = 0;
		for (; next_element_to_insert < remaining_number_elements_to_insert; ++next_element_to_insert) {
			std::swap((*pointcloud_)[next_insert_position_++], *first++);
		}
	}

	return true; // swapping performed
}


template<typename PointT>
void CircularBufferPointCloud<PointT>::insertReverse(const PointT& new_element) {
	if (pointcloud_->empty()) { // filling the buffer
		pointcloud_->push_back(new_element);
		++next_insert_position_;
	} else { // replace last element inserted
		if (next_insert_position_ == 0) {
			pointcloud_->back() = new_element;
		} else {
			(*pointcloud_)[next_insert_position_ - 1] = new_element;
		}
	}
}


template<typename PointT>
void CircularBufferPointCloud<PointT>::insertReverse(const pcl::PointCloud<PointT>& new_elements) {
	if (new_elements.size() < max_buffer_size_) {
		insertReverse(new_elements.begin(), new_elements.end());
	} else {
		insertReverse(new_elements.begin(), new_elements.begin() + max_buffer_size_);
	}
}


template<typename PointT>
void CircularBufferPointCloud<PointT>::insertReverse(typename pcl::PointCloud<PointT>::const_iterator first, typename pcl::PointCloud<PointT>::const_iterator last) {
	size_t number_elements_to_insert = std::distance(first, last);
	if (number_elements_to_insert > max_buffer_size_) {
		last = first + max_buffer_size_;
	}
	size_t remaining_number_elements_to_insert = fillBuffer(number_elements_to_insert, first);
	std::advance(first, number_elements_to_insert - remaining_number_elements_to_insert);

	size_t next_insert_position_reverse = next_insert_position_;
	if (remaining_number_elements_to_insert > 0) {
		if (remaining_number_elements_to_insert <= next_insert_position_reverse) { // replacing old elements
			for (size_t i = 0; i < remaining_number_elements_to_insert; ++i) {
				(*pointcloud_)[next_insert_position_reverse--] = *first++;
			}
		} else { // replacing old elements with wrap around
			size_t next_element_to_insert = 0;
			for (; next_element_to_insert <= next_insert_position_; ++next_element_to_insert) {
				(*pointcloud_)[next_insert_position_reverse--] = *first++;
			}

			next_insert_position_reverse = pointcloud_->size() - 1;
			for (; next_element_to_insert < remaining_number_elements_to_insert; ++next_element_to_insert) {
				(*pointcloud_)[next_insert_position_reverse--] = *first++;
			}
		}
	}
}


template<typename PointT>
void CircularBufferPointCloud<PointT>::eraseNewest(size_t count) {
	if (count > 0 && !pointcloud_->empty()) {
		if (count >= pointcloud_->size()) {
			pointcloud_->clear();
			next_insert_position_ = 0;
		} else if (count <= next_insert_position_) { // remove newest elements
			pointcloud_->erase(pointcloud_->begin() + (next_insert_position_ - count), pointcloud_->begin() + next_insert_position_);
			next_insert_position_ -= count;
		} else { // remove newest elements with wrap around
			if (next_insert_position_ != 0) {
				pointcloud_->erase(pointcloud_->begin(), pointcloud_->begin() + next_insert_position_);
			}

			pointcloud_->erase(pointcloud_->end() - (count - next_insert_position_), pointcloud_->end());
			next_insert_position_ = 0;
		}
	}
}

template<typename PointT>
void CircularBufferPointCloud<PointT>::eraseOldest(size_t count) {
	if (count > 0 && !pointcloud_->empty()) {
		if (count >= pointcloud_->size()) {
			pointcloud_->clear();
			next_insert_position_ = 0;
		} else if (next_insert_position_ + count <= pointcloud_->size()) { // remove oldest elements
			pointcloud_->erase(pointcloud_->begin() + next_insert_position_, pointcloud_->begin() + (next_insert_position_ + count));
		} else { // remove oldest elements with wrap around
			pointcloud_->erase(pointcloud_->begin() + next_insert_position_, pointcloud_->end());
			size_t number_elements_to_remove_at_beginning = count - (pointcloud_->size() - next_insert_position_);
			pointcloud_->erase(pointcloud_->begin(), pointcloud_->begin() + number_elements_to_remove_at_beginning);
			next_insert_position_ -= number_elements_to_remove_at_beginning;
		}
	}
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </CircularBufferPointCloud-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <template instantiations>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </template instantiations>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

} /* namespace dynamic_robot_localization */


