cmake_minimum_required(VERSION 2.8.3)
project(dynamic_robot_localization)



#######################################################################################################################
## packages
#######################################################################################################################

# remove logging (ROSCONSOLE_SEVERITY_NONE=5)
# set(ROS_COMPILE_FLAGS "-DROSCONSOLE_MIN_SEVERITY=5 ${ROS_COMPILE_FLAGS}")

set(${PROJECT_NAME}_CATKIN_COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    pcl_conversions
    pcl_ros
    tf2
    tf2_ros
    rosconsole
    angles
    dynamic_reconfigure
    cmake_modules
    laserscan_to_pointcloud
    message_generation
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_COMPONENTS})
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)



#######################################################################################################################
## catkin specific configuration
#######################################################################################################################


add_message_files(
  FILES
  LocalizationDetailed.msg
  LocalizationDiagnostics.msg
  LocalizationTimes.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)


# generate_dynamic_reconfigure_options(
#    cfg/PlanarLocalization.cfg
# )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
        drl_common
        drl_pose_to_tf_publisher
        drl_cloud_filters
        drl_normal_estimators
        drl_keypoint_detectors
        drl_cloud_matchers
        drl_transformation_validators
        drl_outliers_detectors
        drl_localization
    CATKIN_DEPENDS
        ${${PROJECT_NAME}_CATKIN_COMPONENTS}
        message_runtime
    DEPENDS
        Eigen
        Boost
        PCL
)



#######################################################################################################################
## build
#######################################################################################################################

#==================
# includes
#==================

include_directories(
    include
    ${Eigen_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)



#==================
# libraries
#==================

add_library(drl_common
    src/common/configurable_object.cpp
    src/common/circular_buffer_pointcloud.cpp
    src/common/cloud_publisher.cpp
    src/common/pointcloud_conversions.cpp
    src/common/pointcloud2_builder.cpp
    src/common/registration_visualizer.cpp
    src/common/time_utils.cpp
    src/common/performance_timer.cpp
)

add_library(drl_pose_to_tf_publisher
    src/localization_publisher/pose_to_tf_publisher.cpp
)

add_library(drl_cloud_filters
    src/cloud_filters/cloud_filter.cpp
    src/cloud_filters/voxel_grid.cpp
    src/cloud_filters/pass_through.cpp
)

add_library(drl_normal_estimators
    src/normal_estimators/normal_estimator.cpp
    src/normal_estimators/normal_estimator_sac.cpp
    src/normal_estimators/normal_estimation_omp.cpp
    src/normal_estimators/moving_least_squares.cpp
)

add_library(drl_keypoint_detectors
    src/cloud_matchers/feature_matchers/keypoint_detectors/keypoint_detector.cpp
    src/cloud_matchers/feature_matchers/keypoint_detectors/intrinsic_shape_signature_3d.cpp
    src/cloud_matchers/feature_matchers/keypoint_detectors/sift_3d.cpp
)

add_library(drl_keypoint_descriptors
    src/cloud_matchers/feature_matchers/keypoint_descriptors/keypoint_descriptor.cpp
    src/cloud_matchers/feature_matchers/keypoint_descriptors/keypoint_descriptor_from_normals.cpp
    src/cloud_matchers/feature_matchers/keypoint_descriptors/pfh.cpp
    src/cloud_matchers/feature_matchers/keypoint_descriptors/fpfh.cpp
    src/cloud_matchers/feature_matchers/keypoint_descriptors/shot.cpp
    src/cloud_matchers/feature_matchers/keypoint_descriptors/shape_context_3d.cpp
    src/cloud_matchers/feature_matchers/keypoint_descriptors/unique_shape_context.cpp
    src/cloud_matchers/feature_matchers/keypoint_descriptors/esf.cpp
#   src/cloud_matchers/feature_matchers/keypoint_descriptors/spin_image.cpp
)

add_library(drl_cloud_matchers
    src/cloud_matchers/cloud_matcher.cpp
    src/cloud_matchers/point_matchers/iterative_closest_point.cpp
    src/cloud_matchers/point_matchers/iterative_closest_point_non_linear.cpp
    src/cloud_matchers/point_matchers/iterative_closest_point_with_normals.cpp
    src/cloud_matchers/point_matchers/iterative_closest_point_generalized.cpp
    src/cloud_matchers/point_matchers/normal_distributions_transform_2d.cpp
    src/cloud_matchers/point_matchers/normal_distributions_transform_3d.cpp
    src/cloud_matchers/feature_matchers/feature_matcher.cpp
    src/cloud_matchers/feature_matchers/ia_ransac.cpp
    src/cloud_matchers/feature_matchers/sample_consensus_initial_alignment.cpp
    src/cloud_matchers/feature_matchers/sample_consensus_prerejective.cpp
    src/cloud_matchers/feature_matchers/sample_consensus_initial_alignment_prerejective.cpp
)

add_library(drl_transformation_validators
    src/transformation_validators/transformation_validator.cpp
    src/transformation_validators/euclidean_transformation_validator.cpp
)

add_library(drl_outliers_detectors
    src/outlier_detectors/outlier_detector.cpp
    src/outlier_detectors/euclidean_outlier_detector.cpp
)

add_library(drl_localization
    src/localization/localization.cpp
)



#==================
# executables
#==================

add_executable(drl_pose_to_tf_publisher_node
    src/localization_publisher/pose_to_tf_publisher_node.cpp
)

add_executable(drl_localization_node
    src/localization/localization_node.cpp
)

add_executable(drl_mesh_to_pcd
    src/tools/mesh_to_pcd.cpp
)


#==================
# dependencies
#==================

# add_dependencies(drl_localization ${PROJECT_NAME}_gencfg)
add_dependencies(drl_localization
    ${PROJECT_NAME}_generate_messages_cpp
)


#==================
# library link
#==================

target_link_libraries(drl_common
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

target_link_libraries(drl_pose_to_tf_publisher
    ${catkin_LIBRARIES}
)

target_link_libraries(drl_pose_to_tf_publisher_node
    drl_pose_to_tf_publisher
    ${catkin_LIBRARIES}
)

target_link_libraries(drl_cloud_filters
    drl_common
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

target_link_libraries(drl_normal_estimators
    drl_common
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

target_link_libraries(drl_keypoint_detectors
    drl_common
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

target_link_libraries(drl_keypoint_descriptors
    drl_common
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

target_link_libraries(drl_cloud_matchers
    drl_common
    drl_keypoint_descriptors
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

target_link_libraries(drl_transformation_validators
    drl_common
    ${catkin_LIBRARIES}
)

target_link_libraries(drl_outliers_detectors
    drl_common
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

target_link_libraries(drl_localization
    drl_common
    drl_pose_to_tf_publisher
    drl_cloud_filters
    drl_normal_estimators
    drl_keypoint_detectors
    drl_keypoint_descriptors
    drl_cloud_matchers
    drl_transformation_validators
    drl_outliers_detectors
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

target_link_libraries(drl_localization_node
    drl_localization
    ${catkin_LIBRARIES}
)

target_link_libraries(drl_mesh_to_pcd
    drl_common
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

