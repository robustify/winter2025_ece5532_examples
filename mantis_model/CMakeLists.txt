cmake_minimum_required(VERSION 3.0.2)
project(mantis_model)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  tf2_ros
  dynamic_reconfigure
)

# These commands are required to build custom Gazebo plugins
find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_STANDARD 17)

generate_dynamic_reconfigure_options(
  cfg/MantisPlugin.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(mantis_interface_plugin
  src/MantisInterfacePlugin.cpp
  src/DcMotorSim.cpp
  src/MotorController.cpp
)
add_dependencies(mantis_interface_plugin
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(mantis_interface_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
