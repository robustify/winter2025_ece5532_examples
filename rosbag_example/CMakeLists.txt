cmake_minimum_required(VERSION 3.0.2)
project(rosbag_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(data_generator_node
  src/data_generator_node.cpp
  src/DataGeneratorNode.cpp
)
add_dependencies(data_generator_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(data_generator_node
  ${catkin_LIBRARIES}
)

add_executable(time_analysis_node
  src/time_analysis_node.cpp
  src/TimeAnalysisNode.cpp
)
add_dependencies(time_analysis_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(time_analysis_node
  ${catkin_LIBRARIES}
)

