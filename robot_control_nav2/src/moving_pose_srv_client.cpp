#include "robot_control_nav2/moving_pose_srv_client.hpp"



MovingPoseSrvClient::MovingPoseSrvClient()
: Node("moving_pose_srv_client_node")
{  

RCLCPP_INFO_STREAM(this->get_logger(), "moving_pose_srv_client_node");

   _timer = this->create_wall_timer(std::chrono::milliseconds(100), 
                                    std::bind(&MovingPoseSrvClient::timer_callback, this));
    _response_sub = this->create_subscription<std_msgs::msg::Int32>("response", 10, 
              std::bind(&MovingPoseSrvClient::response_callback, this, std::placeholders::_1));



//   excute_main();

}


void MovingPoseSrvClient::excute_main()
{
  while (rclcpp::ok()) {

      
      RCLCPP_INFO(this->get_logger(), "Waiting..!");
          std_msgs::msg::Int32 a;
    std_msgs::msg::Int32 b;
    a.data = 5 ;
    b.data = 10;

    RCLCPP_INFO_STREAM(this->get_logger(), "sssss");

    this->call_operator_two_ints(a, b);


}
}

void MovingPoseSrvClient::operator_two_ints_request(std_msgs::msg::Int32 a, std_msgs::msg::Int32 b)
{
    rclcpp::Client<robot_msgs::srv::OperatorTwoInts>::SharedPtr client = this->create_client<robot_msgs::srv::OperatorTwoInts>("operator_two_ints");
    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_INFO(this->get_logger(), "Waiting..!");
    }
    auto request = std::make_shared<robot_msgs::srv::OperatorTwoInts::Request>();
    request->a = a.data;
    request->b = b.data;
 
    auto f = client->async_send_request(request);
    try
    {
        auto response = f.get();
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}

void MovingPoseSrvClient::call_operator_two_ints(std_msgs::msg::Int32 a, std_msgs::msg::Int32 b)
{
    std::thread thread = std::thread(std::bind(&MovingPoseSrvClient::operator_two_ints_request, this, a, b));
    thread.detach();
}

void MovingPoseSrvClient::timer_callback()
{
    std_msgs::msg::Int32 a;
    std_msgs::msg::Int32 b;
    a.data = 5 ;
    b.data = 10;

    RCLCPP_INFO_STREAM(this->get_logger(), "sssss");

    this->call_operator_two_ints(a, b);
}



void MovingPoseSrvClient::response_callback(const std_msgs::msg::Int32 &msg)
{
    RCLCPP_INFO(this->get_logger(), "response: = %d", msg.data);
}

int main(int argc, char * argv[])
{
   rclcpp::init(argc, argv);
 
     auto node = std::make_shared<MovingPoseSrvClient>();
     rclcpp::spin(node);
     rclcpp::shutdown();
     return 0;

  return 0;
}