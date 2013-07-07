cmake_minimum_required(VERSION 2.8.3)
project(hubo_forward_kinematics)


find_package(Eigen REQUIRED)

find_package(catkin REQUIRED COMPONENTS kdl_parser message_generation roscpp rospy moveit_msgs std_msgs tf)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  moveit_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_solver
  CATKIN_DEPENDS kdl_parser message_runtime roscpp rospy std_msgs moveit_msgs tf
  DEPENDS system_lib
)

include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})

include_directories(include ${catkin_INCLUDE_DIRS})


add_library(${PROJECT_NAME}_solver src/robot_state_publisher.cpp src/treefksolverposfull_recursive.cpp)
target_link_libraries(${PROJECT_NAME}_solver ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_server src/joint_state_listener.cpp)
target_link_libraries(${PROJECT_NAME}_server ${PROJECT_NAME}_solver log4cxx)

