#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "serial/serial.h"

const int sensor_count = 3;

class SerialToRosNode : public rclcpp::Node {
public:
    SerialToRosNode() : Node("serial_to_ros") {
        // Initialize serial port
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        ser.setTimeout(timeout);

        // Open serial port
        try {
            ser.open();
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
        }

        // Initialize publishers
        for (int i = 0; i < sensor_count; ++i) {
            pub_list.push_back(this->create_publisher<std_msgs::msg::String>("sensor_" + std::to_string(i), 10));
        }

        // Create LED Control publisher
        led_control_pub = this->create_publisher<std_msgs::msg::String>("led_control", 10);

        // // Create LED Control subscriber
        // led_control_sub = this->create_subscription<std_msgs::msg::String>("led_control",
        //     10, std::bind(&SerialToRosNode::ledControlCallback, this, std::placeholders::_1));

        // Create thread to read data from serial
        distance_thread = std::thread(&SerialToRosNode::readDistance, this);
    }

private:
    serial::Serial ser;
    std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> pub_list;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr led_control_pub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr led_control_sub;
    std::thread distance_thread;

    void readDistance() {
        while (rclcpp::ok()) {
            if (ser.available() > 0) {
                std::string data = ser.readline();
                std::vector<std::string> sensor_data = splitString(data, '\t');
                for (size_t i = 0; i < sensor_count && i < sensor_data.size(); ++i) {
                    std_msgs::msg::String msg;
                    msg.data = sensor_data[i];
                    pub_list[i]->publish(msg);
                }
            }
        }
    }

    void ledControlCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::string command = msg->data;
        ser.write(command);
    }

    std::vector<std::string> splitString(const std::string &s, char delimiter) {
        std::vector<std::string> tokens;
        std::string token;
        std::istringstream tokenStream(s);
        while (std::getline(tokenStream, token, delimiter)) {
            tokens.push_back(token);
        }
        return tokens;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialToRosNode>());
    rclcpp::shutdown();
    return 0;
}
