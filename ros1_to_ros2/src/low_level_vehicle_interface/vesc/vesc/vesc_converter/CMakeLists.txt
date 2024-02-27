cmake_minimum_required(VERSION 3.5)
project(vesc_converter)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vesc_msgs REQUIRED)

# Add include directories
include_directories(include)

# Create an executable
add_executable(${PROJECT_NAME}
  src/vesc_converter.cpp
  src/vesc_reception_converter.cpp
)

# Link against the necessary libraries
target_link_libraries(${PROJECT_NAME}
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${vesc_msgs_LIBRARIES}
)

# Install the executable
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Install other files (if any)
# install(FILES
#   # list other files to install
#   DESTINATION share/${PROJECT_NAME}
# )

# Install launch files (if any)
# install(DIRECTORY launch/
#   DESTINATION share/${PROJECT_NAME}/launch
# )

# Install dependencies
ament_export_dependencies(
  rclcpp
  std_msgs
  vesc_msgs
)

# Define export targets (if any)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
