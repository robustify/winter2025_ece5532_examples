cmake_minimum_required(VERSION 3.0.2)
project(roundbot_local_planner)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  costmap_2d
  tf2_ros
  nav_core
  pluginlib
  angles
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(${PROJECT_NAME}
  src/RoundbotLocalPlanner.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

### Install Rules
install(
  TARGETS
    ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  FILES
    plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
