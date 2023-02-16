cmake_minimum_required(VERSION 3.0.2)
project(dead_reckoning_odom)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

add_executable(${PROJECT_NAME}
  src/dead_reckoning_node.cpp
  src/DeadReckoningOdom.cpp
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
