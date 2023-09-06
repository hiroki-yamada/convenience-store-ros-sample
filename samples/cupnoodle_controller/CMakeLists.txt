cmake_minimum_required(VERSION 2.8.3)
project(cupnoodle_controller)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sigverse_ros_bridge
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs 
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)


link_directories(/usr/local/lib)

add_executable(cupnoodle_controller src/cupnoodle_controller.cpp)
target_link_libraries(cupnoodle_controller ${catkin_LIBRARIES})
