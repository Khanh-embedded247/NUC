cmake_minimum_required(VERSION 3.5)
project(vesc_ackermann)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(vesc_msgs REQUIRED)

# Declare the package as an ament package
ament_package()

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${ackermann_msgs_INCLUDE_DIRS}
  ${vesc_msgs_INCLUDE_DIRS}
)

# node executable
add_executable(vesc_to_odom_node src/vesc_to_odom_node.cpp
                                 src/vesc_to_odom.cpp)
ament_target_dependencies(vesc_to_odom_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  ackermann_msgs
  vesc_msgs
)

# node executable
add_executable(ackermann_to_vesc_node src/ackermann_to_vesc_node.cpp
                                      src/ackermann_to_vesc.cpp)
ament_target_dependencies(ackermann_to_vesc_node
  rclcpp
  std_msgs
  ackermann_msgs
  vesc_msgs
)

# nodelet library
add_library(vesc_ackermann_nodelet SHARED
  src/ackermann_to_vesc_nodelet.cpp
  src/ackermann_to_vesc.cpp
  src/vesc_to_odom_nodelet.cpp
  src/vesc_to_odom.cpp)
ament_target_dependencies(vesc_ackermann_nodelet
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  ackermann_msgs
  vesc_msgs
)

# Install targets
install(TARGETS
  vesc_to_odom_node
  ackermann_to_vesc_node
  vesc_ackermann_nodelet
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Install other files
install(FILES
  vesc_ackermann_nodelet.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# TODO: Add testing
