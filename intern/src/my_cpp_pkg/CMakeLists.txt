cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial REQUIRED)

add_executable(rgb src/rgb.cpp)

ament_target_dependencies(rgb rclcpp example_interfaces std_msgs)
target_link_libraries(rgb serial)

install(TARGETS
rgb
DESTINATION lib/${PROJECT_NAME}
)

ament_package()
