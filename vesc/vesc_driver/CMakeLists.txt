cmake_minimum_required(VERSION 3.8)
project(vesc_driver)

add_compile_options(-std=c++17)

if(CMAKE_COMPILER_IS_GNUCXX
   OR CMAKE_CXX_COMPILER_ID
      MATCHES
      "Clang"
)
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(Threads)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

# driver libraries
ament_auto_add_library(
  ${PROJECT_NAME}
  SHARED
  src/vesc_driver.cpp
  src/vesc_interface.cpp
  src/vesc_packet.cpp
  src/vesc_packet_factory.cpp
)
#node executable
add_executable(vesc_driver_node src/vesc_driver_node.cpp)
target_link_libraries(vesc_driver_node vesc_driver)

# Nodelet library
add_library(vesc_driver_nodelet SHARED src/vesc_driver_nodelet.cpp)
target_link_libraries(vesc_driver_nodelet vesc_driver)

ament_target_dependencies(vesc_driver
  rclcpp
  std_msgs
  vesc_msgs
  rosidl_serialization  
)

ament_target_dependencies(vesc_driver_node
  rclcpp
  std_msgs
  vesc_msgs
  rosidl_serialization  
)
ament_target_dependencies(vesc_driver_nodelet
  rclcpp
  std_msgs
  vesc_msgs
  rosidl_serialization
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
ament_auto_package()