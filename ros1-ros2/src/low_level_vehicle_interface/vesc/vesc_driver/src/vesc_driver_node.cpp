/*********************************************************************
khởi tạo node ROS và sử dụng lớp VescDriver để tương tác với driver của VESC 
 ********************************************************************/

#include <rclcpp/rclcpp.hpp>

#include "vesc_driver/vesc_driver.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("vesc_driver_node");
  auto private_nh = rclcpp::Node::make_shared("~");

  vesc_driver::VescDriver vesc_driver(nh, private_nh);

  rclcpp::spin(nh);

  rclcpp::shutdown();
  return 0;
}
