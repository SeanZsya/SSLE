cmake_minimum_required(VERSION 3.0.2)
project(run_onboard)

find_package(catkin REQUIRED COMPONENTS
   roscpp  
   mavros
   std_srvs
   message_generation 

   prometheus_control

   tf
   tf2_ros
   tf2_eigen

   std_msgs
   nav_msgs
   sensor_msgs
   mavros_msgs
   geometry_msgs
   prometheus_msgs
   quadrotor_msgs
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   geometry_msgs 
   nav_msgs
   sensor_msgs
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES px4_command
   CATKIN_DEPENDS mavros roscpp rospy message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(KeyboardEvent src/KeyboardEvent.cpp)

add_executable(pub_cam_pose src/pub_cam_pose.cpp)
add_executable(pub_lidar_pose src/pub_lidar_pose.cpp)
add_executable(pub_px4_cmd src/pub_px4_cmd.cpp)

add_dependencies(pub_cam_pose  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(pub_lidar_pose  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(pub_px4_cmd  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(pub_px4_cmd ${catkin_LIBRARIES})
target_link_libraries(pub_px4_cmd KeyboardEvent)
target_link_libraries(pub_cam_pose ${catkin_LIBRARIES})
target_link_libraries(pub_lidar_pose ${catkin_LIBRARIES})