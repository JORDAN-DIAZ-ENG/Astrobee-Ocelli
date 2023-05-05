cmake_minimum_required(VERSION 2.8.3)
project(registered_cameras)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  pcl_ros
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/registered_cameras_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


