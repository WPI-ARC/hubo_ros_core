cmake_minimum_required(VERSION 2.8.3)
project(hubo_ach_ros_bridge)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS hubo_robot_msgs hubo_sensor_msgs hubo_system_msgs roscpp rospy std_msgs)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  hubo_robot_msgs
  hubo_sensor_msgs
  hubo_system_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hubo_ach_ros_bridge
  CATKIN_DEPENDS hubo_robot_msgs hubo_sensor_msgs hubo_system_msgs roscpp rospy std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Check if hubo.h header has been found
unset(HAVE_HUBO_H CACHE)
check_include_files(hubo.h HAVE_HUBO_H)

# If we have it, add executables
if("${HAVE_HUBO_H}" STREQUAL "")
  message(WARNING "hubo.h not found in path. Bridge executables will not be built.")
else()
  if(${HAVE_HUBO_H} GREATER 0)
    add_executable(feedback src/hubo_ros_feedback.cpp)
    target_link_libraries(feedback ${catkin_LIBRARIES} ach)
    add_dependencies(feedback ${catkin_EXPORTED_TARGETS})
    add_executable(feedforward src/hubo_ros_feedforward_control.cpp)
    target_link_libraries(feedforward ${catkin_LIBRARIES} ach)
    add_dependencies(feedforward ${catkin_EXPORTED_TARGETS})
    message(STATUS "hubo.h found in path. Bridge executables will be built.")
  else()
    message(WARNING "hubo.h not found in path. Bridge executables will not be built.")
  endif()
endif()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  src/hubo_ach_ros_bridge/hubo_joint_state_publisher.py
  src/hubo_ach_ros_bridge/drchubo_joint_state_publisher.py
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
# install(TARGETS hubo_ach_ros_bridge hubo_ach_ros_bridge_node
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

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_hubo_ach_ros_bridge.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
