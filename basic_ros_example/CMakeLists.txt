cmake_minimum_required(VERSION 3.0.2)
project(basic_ros_example)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_service_files(
  FILES
  Adder.srv
)

catkin_python_setup()
generate_messages()
catkin_package()

add_executable(topic_publisher src/topic_publisher.cpp)
target_link_libraries(topic_publisher ${catkin_LIBRARIES})

add_executable(service_advertiser src/service_advertiser.cpp)
add_dependencies(service_advertiser ${PROJECT_NAME}_gencpp)
target_link_libraries(service_advertiser ${catkin_LIBRARIES})

add_executable(service_client src/service_client.cpp)
add_dependencies(service_client ${PROJECT_NAME}_gencpp)
target_link_libraries(service_client ${catkin_LIBRARIES})