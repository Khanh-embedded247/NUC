#include "rclcpp/rclcpp.hpp"
#include "type_custom/srv/operator_two_ints.hpp"
#include <algorithm>

class SortArrayServer : public rclcpp::Node {
public:
    SortArrayServer() : Node("sort_array_server") {
        service_ = this->create_service<type_custom::srv::OperatorTwoInts>(
            "sort_array",
            std::bind(&SortArrayServer::handle_sort_array, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Service is ready to receive requests.");
    }

private:
    void handle_sort_array(
        const std::shared_ptr<type_custom::srv::OperatorTwoInts::Request> request,
        std::shared_ptr<type_custom::srv::OperatorTwoInts::Response> response) {

        // Sắp xếp mảng dựa trên lựa chọn
        switch (request->option) {
            case 1:
                std::sort(request->array.begin(), request->array.end());
                break;
            case 2:
                std::sort(request->array.rbegin(), request->array.rend());
                break;
            case 3:
                // Tìm kiếm phần tử
                float k;
                int i;
                RCLCPP_INFO(this->get_logger(), "Nhập phần tử muốn tìm: ");
                std::cin >> k;
                for (i = 0; i < request->array.size(); i++) {
                    if (request->array[i] == k) {
                        RCLCPP_INFO(this->get_logger(), "Phần tử muốn tìm nằm ở vị trí thứ %d trong mảng.", i);
                    }
                }
                break;
            case 4:
                // Tìm giá trị lớn nhất
                RCLCPP_INFO(this->get_logger(), "Phần tử lớn nhất là: %f", *std::max_element(request->array.begin(), request->array.end()));
                break;
            case 5:
                // Tìm giá trị nhỏ nhất
                RCLCPP_INFO(this->get_logger(), "Phần tử nhỏ nhất là: %f", *std::min_element(request->array.begin(), request->array.end()));
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "Lựa chọn không hợp lệ.");
        }

        // Gán mảng đã sắp xếp vào phản hồi để trả về cho client
        response->sorted_array = request->array;
    }

    rclcpp::Service<type_custom::srv::OperatorTwoInts>::SharedPtr service_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SortArrayServer>());
    rclcpp::shutdown();
    return 0;
}
