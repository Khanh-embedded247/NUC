#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
class Count_Publisher : public rclcpp::Node
{
public:
    Count_Publisher() : Node("count_pub"), count_(0)
    {
        count_publisher = this->create_publisher<std_msgs::msg::String>("pub", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Count_Publisher::timer_callBack, this));
    }
 
private:
    void Input(std::string &handle)
    {
        std::cin >> handle;
    }

    void timer_callBack()
    {
        std::string input;
        Input(input);
        auto message = std_msgs::msg::String();
        message.data ="["+std::to_string(count_++)+"] :"+input ;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        count_publisher->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr count_publisher;
    int count_;
}; 
int main(int argc,char *argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Count_Publisher>());
    rclcpp::shutdown();
    return 0;
}