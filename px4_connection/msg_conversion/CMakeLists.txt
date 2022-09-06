cmake_minimum_required(VERSION 3.0.2)
project(msg_conversion)

find_package(catkin REQUIRED COMPONENTS
   roscpp  
   mavros
   std_srvs 
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
   INCLUDE_DIRS ../common_include
#  LIBRARIES px4_command
   CATKIN_DEPENDS mavros roscpp rospy message_runtime
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ../common_include
)

add_library(KeyboardEvent src/KeyboardEvent.cpp)

add_executable(pub_sensor_pose src/pub_sensor_pose.cpp)
add_dependencies(pub_sensor_pose  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pub_sensor_pose ${catkin_LIBRARIES})

add_executable(pub_px4_cmd src/pub_px4_cmd.cpp)
add_dependencies(pub_px4_cmd  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pub_px4_cmd ${catkin_LIBRARIES} KeyboardEvent)