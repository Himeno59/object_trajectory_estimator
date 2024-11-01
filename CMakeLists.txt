cmake_minimum_required(VERSION 3.0.2)
project(object_trajectory_estimator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pluginlib
  nodelet
  geometry_msgs
  std_msgs
  tf2_msgs
  tf2_geometry_msgs
  tf
  tf2
  tf2_ros
  message_generation
  sensor_msgs
  cv_bridge
  pcl_ros
  pcl_conversions
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

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs   std_msgs   tf2_msgs
)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES basketball_motion
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs sensor_msgs cv_bridge tf2_msgs tf2_geometry_msgs message_runtime nodelet pluginlib
  # DEPENDS system_lib
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
  src/object_trajectory_estimator_nodelet.cpp
  src/recursive_least_square.cpp
  src/least_square.cpp
  src/depth_min.cpp
  src/sphere_fitting.cpp
  )
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )

add_executable(set_rls_matrix
  src/set_rls_matrix.cpp)
target_link_libraries(set_rls_matrix
  ${catkin_LIBRARIES}
  )
