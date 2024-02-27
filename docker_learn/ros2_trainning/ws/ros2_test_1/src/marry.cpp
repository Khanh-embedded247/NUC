#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include<memory>
using std::placeholders::_1;

class Notification : public rclcpp::Node
{
public:
    Notification() : Node("Notification")
    {
        ping_age = this->create_subscription<std_msgs::msg::UInt32>("age", 10, std::bind(&Notification::marry_callBack, this, _1));
    }

private:
    void marry_callBack(const std_msgs::msg::UInt32::SharedPtr msg)
    {
        if (msg->data >= 20)
        {
            RCLCPP_INFO(this->get_logger(), "Old enough to get married");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Old enough to go to prison");
        }
    }

    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr ping_age;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Notification>());
    rclcpp::shutdown();
    return 0;
}
