cmake_minimum_required(VERSION 3.5)
project(vesc_converter)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vesc_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${vesc_msgs_INCLUDE_DIRS}
)

# Declare a C++ executable
add_executable(${PROJECT_NAME} src/vesc_converter.cpp)
add_executable(vesc_reception_converter src/vesc_reception_converter.cpp)

# Specify libraries to link against
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  vesc_msgs
)

ament_target_dependencies(vesc_reception_converter
  rclcpp
  std_msgs
  vesc_msgs
)

# Install targets
install(TARGETS
  ${PROJECT_NAME}
  vesc_reception_converter
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

# Install other files
install(FILES
  # myfile1
  # myfile2
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
