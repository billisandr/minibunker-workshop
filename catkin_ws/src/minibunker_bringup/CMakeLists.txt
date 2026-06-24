cmake_minimum_required(VERSION 3.0.2)
project(minibunker_bringup)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch urdf worlds rviz config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)
