cmake_minimum_required(VERSION 2.8.3)
project(ars_obstacle_avoidance_react)

find_package(catkin REQUIRED COMPONENTS
)

catkin_python_setup()

catkin_package(
  DEPENDS 
  CATKIN_DEPENDS ars_lib_helpers
  INCLUDE_DIRS
  LIBRARIES
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)
