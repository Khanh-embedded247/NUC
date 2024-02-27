cmake_minimum_required(VERSION 3.8)
project(ros2_test_1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(ager src/sum_pub.cpp)
add_executable(notif src/marry.cpp)
ament_target_dependencies(ager rclcpp std_msgs)
ament_target_dependencies(notif rclcpp std_msgs)
install(TARGETS
  ager
  notif
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
