cmake_minimum_required(VERSION 3.0.2)
project(imu_ekf_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/ImuEkfExample.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(imu_ekf_example
  src/imu_ekf_example.cpp
  src/ImuEkfExample.cpp
)
add_dependencies(imu_ekf_example
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(imu_ekf_example
  ${catkin_LIBRARIES}
)
