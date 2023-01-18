cmake_minimum_required(VERSION 3.0.2)
project(transform_examples)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(example1_eigen
  src/example1_eigen.cpp
)

add_executable(example2_eigen
  src/example2_eigen.cpp
)
