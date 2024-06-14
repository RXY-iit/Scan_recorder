cmake_minimum_required(VERSION 3.0.2)
project(scan_recorder)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  sensor_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(scan_recorder_node src/scan_recorder_node.cpp)
add_executable(recorder_map_node src/recoder_map.cpp)

target_link_libraries(recorder_map_node
  ${catkin_LIBRARIES}
)
target_link_libraries(scan_recorder_node
  ${catkin_LIBRARIES}
)
