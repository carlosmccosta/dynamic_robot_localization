cmake_minimum_required(VERSION 2.8.3)
project(dynamic_robot_localization)



##############
## Packages ##
##############

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



###################################
## catkin specific configuration ##
###################################

generate_dynamic_reconfigure_options(
    cfg/PlanarLocalization.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES pose_to_tf_publisher planar_matcher planar_localization
    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_COMPONENTS}
    DEPENDS Eigen Boost PCL
)



###########
## Build ##
###########

include_directories(
    include
    ${Eigen_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_library(pose_to_tf_publisher src/pose_to_tf_publisher.cpp)
add_library(planar_matcher src/planar/planar_matcher.cpp)
add_library(planar_localization src/planar/planar_localization.cpp)

add_executable(pose_to_tf_publisher_node src/pose_to_tf_publisher_node.cpp)
add_executable(planar_localization_node src/planar/planar_localization_node.cpp)

add_dependencies(planar_localization ${PROJECT_NAME}_gencfg)

target_link_libraries(pose_to_tf_publisher ${catkin_LIBRARIES})
target_link_libraries(planar_matcher ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(planar_localization planar_matcher pose_to_tf_publisher ${catkin_LIBRARIES})
target_link_libraries(pose_to_tf_publisher_node pose_to_tf_publisher ${catkin_LIBRARIES})
target_link_libraries(planar_localization_node planar_localization ${catkin_LIBRARIES})

