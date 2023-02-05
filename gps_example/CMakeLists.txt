cmake_minimum_required(VERSION 3.0.2)
project(gps_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  gps_common
)

catkin_python_setup()
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(gps_example
  src/gps_example.cpp
)
target_link_libraries(gps_example
 ${catkin_LIBRARIES}
)

add_executable(ecef_utm_comparison
  src/ecef_utm_comparison
)
target_link_libraries(ecef_utm_comparison
  ${catkin_LIBRARIES}
)