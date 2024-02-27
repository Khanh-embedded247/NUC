cmake_minimum_required(VERSION 3.5)
project(vesc_driver)

add_compile_options(-std=c++14)

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vesc_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(nodelet REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_serialization REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${vesc_msgs_INCLUDE_DIRS}
  ${serial_INCLUDE_DIRS}
  ${nodelet_INCLUDE_DIRS}
  ${pluginlib_INCLUDE_DIRS}
  ${rclcpp_serialization_INCLUDE_DIRS}  # Thêm dòng này
)

# Create the ROS 2 package
ament_package()

###########
## Build ##
###########

# driver libraries
add_library(vesc_driver SHARED
  src/vesc_driver.cpp
  src/vesc_interface.cpp
  src/vesc_packet.cpp
  src/vesc_packet_factory.cpp
)

# node executable
add_executable(vesc_driver_node src/vesc_driver_node.cpp)
target_link_libraries(vesc_driver_node vesc_driver)

# nodelet library
add_library(vesc_driver_nodelet SHARED src/vesc_driver_nodelet.cpp)
target_link_libraries(vesc_driver_nodelet vesc_driver)

ament_target_dependencies(vesc_driver
  rclcpp
  std_msgs
  vesc_msgs
  serial
  rclcpp_serialization
)

ament_target_dependencies(vesc_driver_node
  rclcpp
  std_msgs
  vesc_msgs
  serial
)

ament_target_dependencies(vesc_driver_nodelet
  rclcpp
  std_msgs
  vesc_msgs
  serial
  nodelet
  pluginlib
)

#############
## Install ##
#############

install(TARGETS
  vesc_driver
  vesc_driver_node
  vesc_driver_nodelet
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(FILES vesc_driver_nodelet.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)
