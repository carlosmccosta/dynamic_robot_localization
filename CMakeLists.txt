cmake_minimum_required(VERSION 2.8.3)
project(dynamic_robot_localization)



##############
## Packages ##
##############

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



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS dynamic_robot_localization dynamic_robot_localization_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )



#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dynamic_robot_localization.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
