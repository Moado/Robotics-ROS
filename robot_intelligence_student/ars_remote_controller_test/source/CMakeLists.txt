cmake_minimum_required(VERSION 2.8.3)
project(ars_remote_controller_test)

find_package(catkin REQUIRED COMPONENTS
)

catkin_python_setup()

catkin_package(
  DEPENDS
  CATKIN_DEPENDS
  INCLUDE_DIRS
  LIBRARIES
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)
