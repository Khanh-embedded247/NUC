cmake_minimum_required(VERSION 3.8)
project(robot_control_nav2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)


add_executable(moving_pose_srv_server_node src/moving_pose_srv_server.cpp)
ament_target_dependencies(moving_pose_srv_server_node 
      robot_msgs rclcpp 
      std_msgs geometry_msgs 
      rclcpp_action rclcpp_components
      nav2_msgs
      tf2 tf2_ros
 )

add_executable(moving_pose_srv_client_node src/moving_pose_srv_client.cpp)
ament_target_dependencies(moving_pose_srv_client_node 
      robot_msgs rclcpp 
      std_msgs geometry_msgs 
      rclcpp_action rclcpp_components)

# add_library(moving_pose_server_node SHARED
#   src/moving_pose_server.cpp)

# target_include_directories(moving_pose_server_node PRIVATE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_definitions(moving_pose_server_node
#   PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")

# ament_target_dependencies(moving_pose_server_node
#   "robot_msgs"
#   "rclcpp"
#   "rclcpp_action"
#   "rclcpp_components")
# rclcpp_components_register_node(moving_pose_server_node PLUGIN "robot_control_nav2::FibonacciActionServer" EXECUTABLE moving_pose_server_node)
# install(TARGETS
#   moving_pose_server_node
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin)


# add_library(moving_pose_client_node SHARED
#   src/moving_pose_client.cpp)

# target_include_directories(moving_pose_client_node PRIVATE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_definitions(moving_pose_client_node
#   PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
# ament_target_dependencies(moving_pose_client_node
#   "robot_msgs"
#   "rclcpp"
#   "rclcpp_action"
#   "rclcpp_components")
# rclcpp_components_register_node(moving_pose_client_node PLUGIN "robot_control_nav2::FibonacciActionClient" EXECUTABLE moving_pose_client_node)
# install(TARGETS
#   moving_pose_client_node
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin)



include_directories(./include/)
include_directories(/usr/include/ )
install(TARGETS
    moving_pose_srv_server_node
    moving_pose_srv_client_node
  DESTINATION lib/${PROJECT_NAME})





if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
