#include "std_msgs/msg/u_int32.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>

class Input_person : public rclcpp::Node//class Input_person kế thừa từ rclcpp::Node.Lớp này đại diện 1 node trong ros2
{
public://Tất cả các thành viên (members) và phương thức (methods) sau đó sẽ được truy cập từ bên ngoài lớp.
    Input_person() : Node("age_pub")//hàm khởi tạo lớp Input_person.onstructor của lớp cơ sở (Node) với tên của nút là "age_pub"
    {
        age_pub = this->create_publisher<std_msgs::msg::UInt32>("age", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),//Tạo một timer (timer_) với chu kỳ 500 milliseconds
            std::bind(&Input_person::age_callBack, this));
    }

private://chỉ được truy cập từ bên trong  lớp Input_person ,k thể truy cập bênb ngoài lớp
    void age_callBack()//phương thưucs ẩn
    {
        std::uint32_t input;
        std::cin >> input;
        
        auto message = std_msgs::msg::UInt32();
        message.data = input; 

        RCLCPP_INFO(this->get_logger(), "Please enter your age: '%u'", message.data);//tham chiếu đến đối tượng của lớp INput_person
        age_pub->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr age_pub;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);//khởi tạo ros2 và cung cấp mt cần thiết cho các node
    rclcpp::spin(std::make_shared<Input_person>());
    rclcpp::shutdown();//sử dụng để đóng và giải phóng tất cả các tài nguyên mà ROS 2 đã sử dụng.
    return 0;
}
