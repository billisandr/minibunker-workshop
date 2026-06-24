cmake_minimum_required(VERSION 3.0.2)
project(minibunker_bringup)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/arena_setup.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch urdf worlds rviz config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)
