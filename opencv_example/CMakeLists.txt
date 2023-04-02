cmake_minimum_required(VERSION 3.0.2)
project(opencv_example)

set(${PROJECT_NAME}_CATKIN_DEPS
  dynamic_reconfigure
  cv_bridge
  roscpp
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})
find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/HoughTransform.cfg
)

catkin_package(
  DEPENDS OpenCV
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(hough_transform_node
  src/hough_transform_node.cpp
  src/HoughTransform.cpp
)
add_dependencies(hough_transform_node
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(hough_transform_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
