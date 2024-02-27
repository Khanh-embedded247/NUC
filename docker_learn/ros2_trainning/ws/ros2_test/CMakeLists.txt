cmake_minimum_required(VERSION 3.8)
project(ros2_test)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(talker src/publisher.cpp)
add_executable(header src/subscriber.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
ament_target_dependencies(header rclcpp std_msgs)
install(TARGETS
  talker
  header
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
