#include "rclcpp/rclcpp.hpp"
//Incude file header chứa các định nghĩa kiểu dữ liệu của mesage và sẻvice đã tạo 
#include "type_custom/srv/operator_two_ints.hpp"
//thư viện dùngg để nhập xuất mảng động 
#include <iostream>
#include <vector>
//lớp SortArrayClient kế thừa từ lớp Node 
class SortArrayClient : public rclcpp::Node {
public:
    SortArrayClient() : Node("sort_array_client") {//hàm khởi tạo lớp SortArrayClient ,nơi tạo ra node sort_array_client
       //cách tạo client gửi yêu cầu tới 1 service có tên sort_array với kiểu dữ liệu custom là  type_custom::srv::OperatorTwoInts
        client_ = this->create_client<type_custom::srv::OperatorTwoInts>("sort_array");
    //vòng lặp kiểm tra xem service có sẵn để gửi yêu cầu không .Vòng sẽ chạy đến khi service sẵn sàng hoặc ros2 dừng
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Hệ thống đã chết. Thoát..");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Hệ thống không khả thi. Load lại..");
        }
    //gửi yêu cầu tới service sau khi đã kiểm tra service sẵn sàng 
        send_request();
    }

private:
    void send_request() {
        //Tạo một yêu cầu mới với kiểu dữ liệu type_custom::srv::OperatorTwoInts::Request.
        auto request = std::make_shared<type_custom::srv::OperatorTwoInts::Request>();
        // std::vector<float> arr;

        // Nhập mảng và lựa chọn từ bàn phím
        RCLCPP_INFO(this->get_logger(), "Nhập số phần tử của mảng: ");
        int n;
        std::cin >> n;

        RCLCPP_INFO(this->get_logger(), "Nhập các phần tử của mảng: ");
        for (int i = 0; i < n; ++i) {
            float element;
            std::cin >> element;
            //Thêm các phần tử của mảng được nhập từ bàn phím vào yêu cầu.
            request->array.push_back(element);
        }

        RCLCPP_INFO(this->get_logger(), "Nhập lựa chọn (1-5): ");
        int option;
        std::cin >> option;

        // Convert elements of arr to double
        // std::vector<double> double_arr(arr.begin(), arr.end());

        // Thiết lập yêu cầu requets
        // request->array = double_arr;
        request->option = option;
        
        // Gửi yêu cầu đến service
        auto result = client_->async_send_request(request);

        // Đợi và xử lý kết quả
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            auto sorted_array = result.get()->sorted_array;
            RCLCPP_INFO(this->get_logger(), "Mảng sau khi sắp xếp: ");
            for (float element : sorted_array) {
                RCLCPP_INFO(this->get_logger(), "%f", element);
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service");
        }
    }

    rclcpp::Client<type_custom::srv::OperatorTwoInts>::SharedPtr client_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);//khởi tạo ros2
    rclcpp::spin(std::make_shared<SortArrayClient>());//tạo 1 instance của lớp SortArrayClient và bắt đầu vòng lặp sử lý các sự kiện  
    rclcpp::shutdown();//đóng ros2 sau khi vòng lặp kết thúc 
    return 0;//trả về giá trị 0để để kết thúc chương trình 
}
