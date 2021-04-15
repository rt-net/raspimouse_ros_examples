cmake_minimum_required(VERSION 2.8.3)
project(raspimouse_ros_examples)

find_package(catkin REQUIRED COMPONENTS
  roslint
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

file(GLOB python_scripts scripts/*.py)
catkin_install_python(
  PROGRAMS ${python_scripts}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

roslint_python()
roslint_add_test()