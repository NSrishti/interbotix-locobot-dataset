cmake_minimum_required(VERSION 2.8.3)
project(locobot_simulation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy  
  controller_manager
  effort_controllers
  gazebo_ros
  gazebo_ros_control
  joint_state_controller
  joint_trajectory_controller
  joint_state_publisher
  joint_state_publisher_gui
  robot_state_publisher
  roslaunch
  sensor_msgs
  rviz
  tf2_ros
  xacro
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Model.msg
  LogicalImage.msg
  DetectedObject.msg
  DetectedObjects.msg
  BoundingBox.msg
  BoundingBoxes.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
  INCLUDE_DIRS plugins
  CATKIN_DEPENDS
  roscpp
  rospy
  controller_manager
  effort_controllers
  gazebo_ros
  gazebo_ros_control
  joint_state_controller
  joint_trajectory_controller
  joint_state_publisher
  joint_state_publisher_gui
  robot_state_publisher
  roslaunch
  rviz
  tf2_ros
  xacro
  message_runtime
  sensor_msgs
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/plugins
)

add_subdirectory(${PROJECT_SOURCE_DIR}/plugins)

catkin_install_python(PROGRAMS
    scripts/perception/detect.py
    scripts/movements/look_around.py
    scripts/movements/teleoperation.py
    scripts/stream_reasoning/stream_creator.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)