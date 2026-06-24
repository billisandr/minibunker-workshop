cmake_minimum_required(VERSION 3.0.2)
project(minibunker_behavior)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  vision_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  src/behavior_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
