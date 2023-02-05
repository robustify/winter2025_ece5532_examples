cmake_minimum_required(VERSION 3.0.2)
project(marker_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  tf2_ros
  visualization_msgs
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/MarkerExample.cfg
)

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

add_executable(tf_broadcaster
  src/tf_broadcaster.cpp
)
add_dependencies(tf_broadcaster
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(tf_broadcaster
  ${catkin_LIBRARIES}
)
