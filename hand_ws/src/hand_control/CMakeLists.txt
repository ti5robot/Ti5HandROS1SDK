cmake_minimum_required(VERSION 3.0.2)
project(hand_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(keyboard_publisher src/keyboard_publisher.cpp)
target_link_libraries(keyboard_publisher ${catkin_LIBRARIES})

add_executable(hand_control_node src/hand_control_node.cpp)
target_link_libraries(hand_control_node ${catkin_LIBRARIES})