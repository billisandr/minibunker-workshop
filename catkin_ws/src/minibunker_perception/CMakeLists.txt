cmake_minimum_required(VERSION 3.0.2)
project(minibunker_perception)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  vision_msgs
  cv_bridge
)

catkin_package()

catkin_install_python(PROGRAMS
  src/detector_node.py
  src/pi_camera_node.py
  src/image_pub_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
