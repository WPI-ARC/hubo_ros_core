cmake_minimum_required(VERSION 2.8.3)
project(hubo_trajectory_interface)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS actionlib actionlib_msgs hubo_robot_msgs hubo_components hubo_components roscpp rospy trajectory_msgs)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

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
  actionlib_msgs
  hubo_robot_msgs
  trajectory_msgs
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
#  LIBRARIES hubo_trajectory_interface
  CATKIN_DEPENDS actionlib actionlib_msgs hubo_robot_msgs roscpp rospy trajectory_msgs hubo_components hubo_components
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

# Add executables
add_executable(hubo_trajectory_action src/hubo_trajectory_action.cpp)
target_link_libraries(hubo_trajectory_action ${catkin_LIBRARIES})
add_dependencies(hubo_trajectory_action hubo_robot_msgs_generate_message_cpp)


add_executable(hubo_trajectory_compromise_interface src/hubo_trajectory_compromise_interface.cpp)
target_link_libraries(hubo_trajectory_compromise_interface ${catkin_LIBRARIES} ach)
add_dependencies(hubo_trajectory_compromise_interface hubo_robot_msgs_generate_message_cpp)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS hubo_trajectory_interface hubo_trajectory_interface_node
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
install(FILES
  launch/hubo_full_body_trajectory_action_node.launch
  launch/hubo_full_body_trajectory_controller_node.launch
  launch/hubo_full_body_trajectory_interface.launch
  config/hubo_fullbody_interface.yaml
  config/hubo_fullbody_interface_wfingers.yaml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_hubo_trajectory_interface.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
