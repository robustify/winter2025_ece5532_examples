cmake_minimum_required(VERSION 3.0.2)
project(gnss_ekf_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/GnssEkfExample.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(gnss_ekf_example
  src/gnss_ekf_example.cpp
  src/GnssEkfExample.cpp
)
add_dependencies(gnss_ekf_example
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(gnss_ekf_example
  ${catkin_LIBRARIES}
)
