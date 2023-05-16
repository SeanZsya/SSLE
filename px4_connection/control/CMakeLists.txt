cmake_minimum_required(VERSION 2.8.3)
project(prometheus_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  mavros
  std_msgs
  std_srvs
  tf
  tf2_ros
  tf2_eigen
  mavros_msgs
  prometheus_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)



################################################
## Declare ROS messages, services and actions ##
################################################

catkin_package(
  INCLUDE_DIRS ../common_include
  CATKIN_DEPENDS  message_runtime
)


###########
## Build ##
###########


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ../common_include
)


###############################
##      声明可执行cpp文件     ##
###############################

add_library(OptiTrackFeedbackRigidBody src/lib/OptiTrackFeedBackRigidBody.cpp)

###### Main File ##########

##px4_pos_estimator.cpp
add_executable(px4_pos_estimator src/px4_pos_estimator.cpp)
add_dependencies(px4_pos_estimator  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 
target_link_libraries(px4_pos_estimator ${catkin_LIBRARIES})
target_link_libraries(px4_pos_estimator OptiTrackFeedbackRigidBody)

add_executable(px4_sender src/px4_sender.cpp)
add_dependencies(px4_sender  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 
target_link_libraries(px4_sender ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)