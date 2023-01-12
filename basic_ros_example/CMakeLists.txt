cmake_minimum_required(VERSION 3.0.2)
project(basic_ros_example)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package()
