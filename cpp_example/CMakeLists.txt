cmake_minimum_required(VERSION 3.0.2)
project(cpp_example)

catkin_package()

add_executable(mult_example
  mult_example.cpp
)

add_executable(factorial_example
  factorial_example.cpp
  Factorial.cpp
)
