cmake_minimum_required(VERSION 3.0.2)
project(marker_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(marker_example_node
  src/marker_example.cpp
)
target_link_libraries(marker_example_node
  ${catkin_LIBRARIES}
)
