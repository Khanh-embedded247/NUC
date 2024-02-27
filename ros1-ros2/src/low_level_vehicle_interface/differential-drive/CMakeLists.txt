cmake_minimum_required(VERSION 3.5)
project(differential_drive)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(message_generation REQUIRED)
find_package(vesc_msgs REQUIRED)

install(TARGETS  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(ament_cmake rclcpp std_msgs message_runtime vesc_msgs)
ament_package()
