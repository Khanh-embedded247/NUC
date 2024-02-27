cmake_minimum_required(VERSION 3.5)
project(pid)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 14)

# Find required ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# Add ROS 2 parameters dependency
find_package(rclcpp_components REQUIRED)

# Include directories
include_directories(
    include
    ${CMAKE_CURRENT_BINARY_DIR}/include  # Include the generated headers
    ${rclcpp_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${rosgraph_msgs_INCLUDE_DIRS}
)

# Build executables
add_executable(controller src/controller.cpp src/pid.cpp)
add_executable(plant_sim src/plant_sim.cpp)
add_executable(setpoint_node src/setpoint_node.cpp)
add_executable(sim_time src/sim_time.cpp)
add_executable(autotune src/autotune.cpp)

# Declare dependencies for each executable
ament_target_dependencies(controller rclcpp std_msgs rosgraph_msgs)
ament_target_dependencies(plant_sim rclcpp std_msgs rosgraph_msgs)
ament_target_dependencies(setpoint_node rclcpp std_msgs)
ament_target_dependencies(sim_time rclcpp std_msgs rosgraph_msgs)
ament_target_dependencies(autotune rclcpp std_msgs)

# Install targets
install(TARGETS controller plant_sim setpoint_node sim_time autotune
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Ament package configuration
ament_package()
