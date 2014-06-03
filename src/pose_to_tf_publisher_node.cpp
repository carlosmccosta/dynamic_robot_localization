/**\file planar_localization_node.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <ros/ros.h>
#include "dynamic_robot_localization/pose_to_tf_publisher.h"
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



// ###################################################################################   <main>   ##############################################################################
int main(int argc, char** argv) {
	ros::init(argc, argv, "pose_to_tf_publisher");

	ros::NodeHandlePtr node_handle(new ros::NodeHandle());
	ros::NodeHandlePtr private_node_handle(new ros::NodeHandle("~"));
	dynamic_robot_localization::PoseToTFPublisher pose_to_tf_publisher;

	pose_to_tf_publisher.readConfigurationFromParameterServer(node_handle, private_node_handle);
	pose_to_tf_publisher.publishInitialPoseFromParameterServer();
	pose_to_tf_publisher.startPublishingTFFromPoseTopics();

	return 0;
}
// ###################################################################################   </main>   #############################################################################
