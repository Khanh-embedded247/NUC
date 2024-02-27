/* hương trình này khởi tạo và duy trì một node ROS với tên "ackermann_to_vesc_node" và
 sử dụng một đối tượng từ gói vesc_ackermann để thực hiện chức năng cụ thể liên quan đến 
 chuyển đổi lệnh điều khiển từ "Ackermann drive" sang dạng VESC*/
#include <rclcpp/rclcpp.hpp>
#include "vesc_ackermann/ackermann_to_vesc.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("ackermann_to_vesc_node");
  auto private_nh = nh->create_sub_node("ackermann_to_vesc_node");

  vesc_ackermann::AckermannToVesc ackermann_to_vesc(nh, private_nh);

  rclcpp::spin(nh);

  rclcpp::shutdown();

  return 0;
}
