#include <rclcpp/rclcpp.hpp>

#include "vesc_ackermann/vesc_to_odom.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("vesc_to_odom_node");
  auto private_nh = rclcpp::Node::make_shared("~");//tạo node con giúp k xung đột tên với node cha ,..

  vesc_ackermann::VescToOdom vesc_to_odom(nh, private_nh);

  rclcpp::spin(nh);

  rclcpp::shutdown();

  return 0;
}
