cmake_minimum_required(VERSION 2.8.3)
project(raspimouse_ros_examples)

find_package(catkin REQUIRED COMPONENTS
  roslint
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

roslint_python()