cmake_minimum_required(VERSION 2.8.3)
project(dynamic_robot_localization)



#######################################################################################################################
## packages
#######################################################################################################################

# remove logging (ROSCONSOLE_SEVERITY_NONE=5)
# set(ROS_COMPILE_FLAGS "-DROSCONSOLE_MIN_SEVERITY=5 ${ROS_COMPILE_FLAGS}")

set(${PROJECT_NAME}_CATKIN_COMPONENTS 
    roscpp
    sensor_msgs
    geometry_msgs
    pcl_conversions
    pcl_ros
    tf2
    tf2_ros
    rosconsole
    dynamic_reconfigure
    cmake_modules
    laserscan_to_pointcloud
    robot_localization_tools
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_COMPONENTS})
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)



#######################################################################################################################
## catkin specific configuration
#######################################################################################################################

# generate_dynamic_reconfigure_options(
#    cfg/PlanarLocalization.cfg
# )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES        
        drl_common
        drl_pose_to_tf_publisher
        drl_cloud_filters
        drl_cloud_matchers
        drl_transformation_validators
        drl_outliers_detectors
        drl_localization
    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_COMPONENTS}
    DEPENDS Eigen Boost PCL
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
    src/common/pointcloud_conversions.cpp
    src/common/pointcloud2_builder.cpp
)

add_library(drl_pose_to_tf_publisher
    src/localization_publisher/pose_to_tf_publisher.cpp
)

add_library(drl_cloud_filters
    src/cloud_filters/cloud_filter.cpp
    src/cloud_filters/voxel_grid.cpp
)

add_library(drl_cloud_matchers
    src/cloud_matchers/cloud_matcher.cpp
    src/cloud_matchers/point_matchers/iterative_closest_point.cpp
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

add_executable(drl_pose_to_tf_publisher_node src/localization_publisher/pose_to_tf_publisher_node.cpp)
add_executable(drl_localization_node src/localization/localization_node.cpp)



#==================
# dependencies
#==================

# add_dependencies(drl_localization ${PROJECT_NAME}_gencfg)


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
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

target_link_libraries(drl_cloud_matchers
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

target_link_libraries(drl_transformation_validators
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

