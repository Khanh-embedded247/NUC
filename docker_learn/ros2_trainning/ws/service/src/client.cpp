#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;//định nghĩa hằng số thời gian với đơn vị thời gian tự động

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 3) {//kiểm tra đôi số dòng lệnh 
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y");
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");//Tạo một đối tượng node với tên "add_two_ints_client" bằng cách sử dụng smart pointer.
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client =
    node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");//Tạo một client để gọi dịch vụ add_two_ints///example_interfaces::srv::AddTwoInts là kiểu dữ liệu của service.

  auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();//Tạo một đối tượng yêu cầu (request) là một đối tượng service request có kiểu example_interfaces::srv::AddTwoInts::Request.
  request->a = atoll(argv[1]);//Lấy giá trị từ dòng lệnh và gán vào yêu cầu request.
  request->b = atoll(argv[2]);

  while (!client->wait_for_service(1s)) {//Chờ đợi cho đến khi dịch vụ sẵn sàng trong thời gian 1 giây. Nếu không sẵn sàng, hiển thị thông báo và tiếp tục chờ.
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);//Gửi yêu cầu đến dịch vụ và nhận kết quả bằng cách sử dụng async_send_request. 
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==// nếu Kết quả (result) là một rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture (future shared pointer).
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }

  rclcpp::shutdown();
  return 0;
}