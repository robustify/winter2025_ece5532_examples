cmake_minimum_required(VERSION 3.0.2)
project(ros_param_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
)

catkin_python_setup()
generate_dynamic_reconfigure_options(
  cfg/ReconfigExample.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(static_param_example
  src/static_param_example.cpp
)
target_link_libraries(static_param_example
  ${catkin_LIBRARIES}
)

add_executable(reconfig_example
  src/reconfig_example.cpp
)
add_dependencies(reconfig_example
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(reconfig_example
  ${catkin_LIBRARIES}
)


