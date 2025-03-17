cmake_minimum_required(VERSION 3.0.2)
project(bouncing_ball_estimator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nodelet
  pluginlib
  tf
  tf2
  tf2_ros
  cv_bridge
  pcl_ros
  pcl_conversions
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2_msgs
  tf2_geometry_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  BallStateStamped.msg
  FbCheck.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetRLSParameters.srv
  GetRLSParameters.srv
  SetRLSMatrix.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs   std_msgs   tf2_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/bouncing_ball_estimator_nodelet.cpp
  src/recursive_least_square.cpp
  src/least_square.cpp
  src/sphere_fitting.cpp
  )
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )

## srv
add_executable(set_rls_matrix
  src/set_rls_matrix.cpp)
target_link_libraries(set_rls_matrix
  ${catkin_LIBRARIES}
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  # CATKIN_DEPENDS roscpp rospy nodelet pluginlib tf tf2 tf2_ros cv_bridge pcl_ros pcl_conversions sensor_msgs geometry_msgs std_msgs tf2_msgs tf2_geometry_msgs message_generation
  # DEPENDS system_lib
)
