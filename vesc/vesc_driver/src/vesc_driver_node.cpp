/*********************************************************************
Mục đích của file này là:

Khởi tạo môi trường ROS 2 (rclcpp::init).
Tạo một Node  tên "vesc_driver_node".
Tạo một Node riêng tư (private_nh) để xử lý các tham số cấu hình riêng tư của nút.
Tạo một đối tượng vesc_driver::VescDriver với Node chính và Node private
Bắt đầu vòng lặp chính của nút với rclcpp::spin.
clear và đóng môi trường ROS 2 khi nút kết thúc (rclcpp::shutdown).
 **************************************************** *******************/

// VIỆC CẦN LÀM: Di chuyển sang ROS2
#include <rclcpp/rclcpp.hpp>

#include "/home/robotic/vesc/vesc_driver/include/vesc_driver/vesc_driver.h"

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

