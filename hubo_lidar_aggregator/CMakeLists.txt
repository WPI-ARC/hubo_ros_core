cmake_minimum_required(VERSION 2.8.3)
project(hubo_lidar_aggregator)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS geometry_msgs laser_geometry pcl pcl_ros message_generation roscpp sensor_msgs std_msgs hubo_sensor_msgs tf)

###################################
## catkin specific configuration ##
###################################

catkin_package(CATKIN_DEPENDS geometry_msgs laser_geometry pcl pcl_ros message_runtime roscpp sensor_msgs std_msgs hubo_sensor_msgs tf DEPENDS system_lib)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

add_executable(hubo_lidar_aggregator_node src/lidar_aggregator_node.cpp)
add_dependencies(hubo_lidar_aggregator_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(hubo_lidar_aggregator_node ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS hubo_lidar_aggregator_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
