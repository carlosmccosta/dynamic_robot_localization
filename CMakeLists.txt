cmake_minimum_required(VERSION 2.8.3)
project(dynamic_robot_localization)



#######################################################################################################################
## packages
#######################################################################################################################

set(${PROJECT_NAME}_CATKIN_COMPONENTS
    angles
    dynamic_reconfigure
    geometry_msgs
    laserscan_to_pointcloud
    message_generation
    pcl_conversions
    pcl_ros
    pose_to_tf_publisher
    rosconsole
    roscpp
    rostime
    sensor_msgs
    std_msgs
    tf2
    tf2_ros
    xmlrpcpp
)

find_package(OpenMP)

if(OPENMP_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
endif()

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_COMPONENTS})
find_package(Eigen3 REQUIRED)
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
   geometry_msgs
   sensor_msgs
   std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
        drl_cloud_analyzers
        drl_cloud_filters
        drl_cloud_matchers
        drl_common
        drl_convergence_estimators
        drl_curvature_estimators
        drl_keypoint_detectors
        drl_keypoint_descriptors
        drl_localization
        drl_normal_estimators
        drl_outliers_detectors
        drl_registration_covariance_estimators
        drl_transformation_validators
    CATKIN_DEPENDS
        ${${PROJECT_NAME}_CATKIN_COMPONENTS}
        message_runtime
    DEPENDS
        Boost
        EIGEN3
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
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)



#==================
# libraries
#==================

add_library(drl_common
    src/common/cumulative_static_transform_broadcaster.cpp
    src/common/configurable_object.cpp
    src/common/circular_buffer_pointcloud.cpp
    src/common/cloud_publisher.cpp
    src/common/cloud_viewer.cpp
    src/common/math_utils.cpp
    src/common/pointcloud_conversions.cpp
    src/common/pointcloud_utils.cpp
    src/common/pointcloud2_builder.cpp
    src/common/registration_visualizer.cpp
    src/common/time_utils.cpp
    src/common/performance_timer.cpp
    src/common/verbosity_levels.cpp
)

add_library(drl_convergence_estimators
    src/convergence_estimators/default_convergence_criteria_with_time.cpp
)

add_library(drl_cloud_filters
    src/cloud_filters/cloud_filter.cpp
    src/cloud_filters/voxel_grid.cpp
    src/cloud_filters/approximate_voxel_grid.cpp
    src/cloud_filters/pass_through.cpp
    src/cloud_filters/radius_outlier_removal.cpp
    src/cloud_filters/crop_box.cpp
    src/cloud_filters/random_sample.cpp
    src/cloud_filters/statistical_outlier_removal.cpp
    src/cloud_filters/scale.cpp
    src/cloud_filters/covariance_sampling.cpp
    src/cloud_filters/plane_segmentation.cpp
    src/cloud_filters/euclidean_clustering.cpp
)

add_library(drl_curvature_estimators
    src/curvature_estimators/curvature_estimator.cpp
    src/curvature_estimators/principal_curvatures_estimation.cpp
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
    src/cloud_matchers/point_matchers/iterative_closest_point_2d.cpp
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

add_library(drl_cloud_analyzers
    src/cloud_analyzers/cloud_analyzer.cpp
    src/cloud_analyzers/angular_distribution_analyzer.cpp
)

add_library(drl_registration_covariance_estimators
    src/registration_covariance_estimators/registration_covariance_estimator.cpp
    src/registration_covariance_estimators/registration_covariance_point_to_point_3d.cpp
    src/registration_covariance_estimators/registration_covariance_point_to_plane_3d.cpp
    src/registration_covariance_estimators/registration_covariance_point_to_point_pm_3d.cpp
    src/registration_covariance_estimators/registration_covariance_point_to_plane_pm_3d.cpp
)

add_library(drl_localization
    src/localization/localization.cpp
)



#==================
# executables
#==================

add_executable(drl_localization_node
    src/localization/localization_node.cpp
)

add_executable(drl_mesh_to_pcd
    src/tools/mesh_to_pcd.cpp
)


#==================
# dependencies
#==================

add_dependencies(drl_cloud_analyzers
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

add_dependencies(drl_cloud_filters
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

add_dependencies(drl_cloud_matchers
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

add_dependencies(drl_common
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

add_dependencies(drl_convergence_estimators
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

add_dependencies(drl_curvature_estimators
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

add_dependencies(drl_keypoint_detectors
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

add_dependencies(drl_keypoint_descriptors
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

add_dependencies(drl_localization
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

add_dependencies(drl_normal_estimators
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

add_dependencies(drl_outliers_detectors
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

add_dependencies(drl_registration_covariance_estimators
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

add_dependencies(drl_transformation_validators
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

add_dependencies(drl_localization_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

add_dependencies(drl_mesh_to_pcd
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)


#==================
# libraries link
#==================

target_link_libraries(drl_common
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

target_link_libraries(drl_convergence_estimators
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

target_link_libraries(drl_cloud_filters
    drl_common
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

target_link_libraries(drl_curvature_estimators
    drl_common
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

target_link_libraries(drl_normal_estimators
    drl_common
    drl_curvature_estimators
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
    drl_convergence_estimators
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

target_link_libraries(drl_cloud_analyzers
    drl_common
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

target_link_libraries(drl_registration_covariance_estimators
    drl_common
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

target_link_libraries(drl_localization
    drl_common
    drl_cloud_filters
    drl_normal_estimators
    drl_keypoint_detectors
    drl_keypoint_descriptors
    drl_cloud_matchers
    drl_transformation_validators
    drl_outliers_detectors
    drl_cloud_analyzers
    drl_registration_covariance_estimators
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)


#==================
# executables link
#==================

target_link_libraries(drl_localization_node
    drl_common
    drl_localization
    ${catkin_LIBRARIES}
)

target_link_libraries(drl_mesh_to_pcd
    drl_common
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

