/**\file pointcloud2_builder.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/common/pointcloud2_builder.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
PointCloud2Builder::PointCloud2Builder() : number_points_in_pointcloud_(0) {
	createNewCloud("");
}

PointCloud2Builder::~PointCloud2Builder() {}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <PointCloud2Builder-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void PointCloud2Builder::createNewCloud(std::string frame_id, size_t number_reserved_points) {
	pointcloud_msg_ = sensor_msgs::PointCloud2Ptr(new sensor_msgs::PointCloud2());
	pointcloud_msg_->header.frame_id = frame_id;
	pointcloud_msg_->height = 1;
	pointcloud_msg_->width = 0;
	pointcloud_msg_->fields.clear();
	pointcloud_msg_->fields.resize(3);
	pointcloud_msg_->fields[0].name = "x";
	pointcloud_msg_->fields[0].offset = 0;
	pointcloud_msg_->fields[0].datatype = sensor_msgs::PointField::FLOAT32;
	pointcloud_msg_->fields[0].count = 1;
	pointcloud_msg_->fields[1].name = "y";
	pointcloud_msg_->fields[1].offset = 4;
	pointcloud_msg_->fields[1].datatype = sensor_msgs::PointField::FLOAT32;
	pointcloud_msg_->fields[1].count = 1;
	pointcloud_msg_->fields[2].name = "z";
	pointcloud_msg_->fields[2].offset = 8;
	pointcloud_msg_->fields[2].datatype = sensor_msgs::PointField::FLOAT32;
	pointcloud_msg_->fields[2].count = 1;
	pointcloud_msg_->point_step = 12;
	pointcloud_msg_->data.reserve(number_reserved_points * pointcloud_msg_->point_step); // reserve memory to avoid reallocations
	number_points_in_pointcloud_ = 0;
}


void PointCloud2Builder::addNewPoint(float x, float y, float z) {
	float point_data[3];
	point_data[0] = x;
	point_data[1] = y;
	point_data[2] = z;
	unsigned char* point_data_bytes = (unsigned char*)(&point_data[0]);
	pointcloud_msg_->data.insert(pointcloud_msg_->data.end(), &point_data_bytes[0], &point_data_bytes[3 * sizeof(float)]);

	++number_points_in_pointcloud_;
}


sensor_msgs::PointCloud2Ptr PointCloud2Builder::getPointcloudMsg() {
	pointcloud_msg_->width = number_points_in_pointcloud_;
	pointcloud_msg_->row_step = pointcloud_msg_->width * pointcloud_msg_->point_step;
	pointcloud_msg_->data.resize(pointcloud_msg_->height * pointcloud_msg_->row_step); // resize to shrink the vector size to the real number of points inserted

	pointcloud_msg_->header.stamp = ros::Time::now();
	return pointcloud_msg_;
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </PointCloud2Builder-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// =============================================================================   <private-section>   =========================================================================
// =============================================================================   </private-section>  =========================================================================

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <template instantiations>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </template instantiations>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

} /* namespace dynamic_robot_localization */

