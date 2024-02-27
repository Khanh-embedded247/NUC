#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <memory>

void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,//con trỏ chỉ biến request từ biến 2 số nguyên
          std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>      response)//con trỏ chỉ giá trị kết quả của phép côngj số nguyên 
{
  response->sum = request->a + request->b;//hàm cộng giá trị 2 số nguyên
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);//khởi tạo ROS2 và cung cấp các môi trường cần thiết cho các node  

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");//Khởi tạo node  

  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
    node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);//create a service cho node và tự động adverises cùng phương thức add

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

  rclcpp::spin(node);
  rclcpp::shutdown();//sử dụng để đongs và giải phóng tất cả tài nguyên đã sử dụng 
}