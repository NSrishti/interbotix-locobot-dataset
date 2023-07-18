cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

# Find Gazebo
find_package(gazebo REQUIRED gazebo_ros gazebo_plugins)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin
#add_library(logcam_plugin SHARED logcam_plugin.cc)
#target_link_libraries(logcam_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})


add_library(logical_camera_plugin SHARED 
  logical_camera_plugin.cpp logical_camera_plugin.h
)
target_link_libraries(logical_camera_plugin 
  ${catkin_LIBRARIES} 
  ${GAZEBO_LIBRARIES}
)