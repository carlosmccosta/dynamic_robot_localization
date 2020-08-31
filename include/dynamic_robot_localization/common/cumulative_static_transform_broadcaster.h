#pragma once


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  <includes> <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// std includes
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

// ROS includes
#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>

// external libs includes
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  </includes> <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


namespace dynamic_robot_localization {
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  <CumulativeStaticTransformBroadcaster>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
class CumulativeStaticTransformBroadcaster {
	public:
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		CumulativeStaticTransformBroadcaster() {}
		virtual ~CumulativeStaticTransformBroadcaster() {}
		static std::shared_ptr< CumulativeStaticTransformBroadcaster > getSingleton(ros::NodeHandlePtr& node_handle);
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		void setup(ros::NodeHandlePtr& node_handle);
		void updateTFCache(const tf2_msgs::TFMessageConstPtr& tf_msg);
		void updateTFCache(const std::vector<geometry_msgs::TransformStamped>& tfs);
		void sendTransform(const geometry_msgs::TransformStamped& tf);
		void sendTransform(const std::vector<geometry_msgs::TransformStamped>& tfs);
		void sendCachedTFs();

	private:
		ros::Subscriber static_tf_subscriber_;
		ros::Publisher static_tf_publisher_;
		std::map< std::string, geometry_msgs::TransformStamped > cached_static_tfs_;
		std::shared_ptr< std::mutex > cached_static_tfs_mutex_;
		static std::shared_ptr< std::mutex > singleton__mutex_;
		static std::shared_ptr< CumulativeStaticTransformBroadcaster > singleton_;
};
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  </CumulativeStaticTransformBroadcaster>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
} /* namespace dynamic_robot_localization */
