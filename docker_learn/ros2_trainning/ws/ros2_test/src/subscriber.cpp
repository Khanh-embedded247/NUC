#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include<string>
#include<memory>
using std::placeholders::_1;
class Count_Subscriber:public rclcpp::Node
{
    public:
    Count_Subscriber():Node("count_sub")
    {
        count_sub_=this->create_subscription<std_msgs::msg::String>("pub",10,std::bind(&Count_Subscriber::sub_CallBack,this, _1));
        

    }
    
    private:
    void sub_CallBack(const std_msgs::msg::String&msg) 
    {
        RCLCPP_INFO(this->get_logger(),"I received : '%s'",msg.data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr count_sub_;
};
int main(int argc,char* argv[]){
    rclcpp::init(argc,argv);
    /*rclcpp::spin: hàm ros2 bắt đầu vòng lặp  chính ros2,giữ mã chạy,xử lí sự kiện và gọi callBack
    std::make_shared<>: tạo 1 smart pointer đến object của class
    */
    rclcpp::spin(std::make_shared<Count_Subscriber>());//bđ vòng lặp chính xử lý sự kiện và callBack từ topic,service,timer.
    rclcpp::shutdown();
    return 0;
}