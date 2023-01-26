cmake_minimum_required(VERSION 3.0.2)
project(ros_time_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_python_setup()
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(time_stamp_example
  src/time_stamp_example.cpp
)
target_link_libraries(time_stamp_example
  ${catkin_LIBRARIES}
)

add_executable(timer_example
  src/timer_example.cpp
)
target_link_libraries(timer_example
  ${catkin_LIBRARIES}
)
