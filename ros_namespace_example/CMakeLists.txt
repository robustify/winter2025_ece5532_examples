cmake_minimum_required(VERSION 3.0.2)
project(ros_namespace_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_python_setup()
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(namespace_example
  src/namespace_example.cpp
)
target_link_libraries(namespace_example
  ${catkin_LIBRARIES}
)
