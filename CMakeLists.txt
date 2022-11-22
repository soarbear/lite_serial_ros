cmake_minimum_required(VERSION 3.0.2)
project(lite_serial_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  lite_serial  src/lite_serial.cpp
)

add_executable(lite_serial_node src/lite_serial_node.cpp)
target_link_libraries(lite_serial_node lite_serial ${catkin_LIBRARIES})
