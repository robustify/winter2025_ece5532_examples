cmake_minimum_required(VERSION 3.0.2)
project(diff_drive_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
)

catkin_python_setup()
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(diff_drive_example
  src/diff_drive_example.cpp
)
target_link_libraries(diff_drive_example
  ${catkin_LIBRARIES}
)
