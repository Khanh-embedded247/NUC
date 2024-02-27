#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <chrono>
#include "example_interfaces/msg/string.hpp"
#include "serial/serial.h"
#include <sstream>
#include "rcutils/types/uint8_array.h"
#include <vector>
#include<thread>
using namespace rclcpp;
const int NUM_SENSOR = 3;
class SensorReader : public rclcpp ::Node // MOdDIFY name
{
public:
    SensorReader() : Node("sensor_reader") // Modify name

    {
        // Setup serial port
        serial_port_.setPort("/dev/ttyUSB0"); // Điều chỉnh thiết bị serial của bạn
        serial_port_.setBaudrate(115200);     // Điều chỉnh baudrate phù hợp
        serial_port_.open();
        for (int i = 0; i < NUM_SENSOR; i++)
        {
            sonars_publisher_.push_back(this->create_publisher<example_interfaces::msg::String>("sensor_" + std::to_string(i), 10));

            // Create a timer for read node data
            timer_ = this->create_wall_timer(std::chrono::microseconds(500),                  // while 1s timercallback had called
                                             std::bind(&SensorReader::readSensorData, this)); // after create timer iss called
        }

        RCLCPP_INFO(this->get_logger(), "Connected to Serial ttyUSB0");
    }

private:
    void readSensorData()
    {
        if (serial_port_.available() > 0)
        {
            std::string data = serial_port_.read(serial_port_.available());
            std::istringstream ss(data);
            std::string word;
            std::vector<std::string> readings;
            while (std::getline(ss, word, '\t'))
            {
                readings.push_back(word);
            }

            if (readings.size() >= sensor_count_)
            {
                for (int i = 0; i < sensor_count_; i++)
                {
                    example_interfaces::msg::String msg;
                    msg.data = readings[i];
                    sonars_publisher_[i]->publish(msg);
                }
            }
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr> sonars_publisher_;
    serial::Serial serial_port_;
    int sensor_count_;
};
class command : public rclcpp::Node
{
public:
    command() : Node("led_command")
    {

        // Create a subscriber to receive commands from the keyboard
        command_subscriber_=this->create_publisher<example_interfaces::msg::String>("led_control" , 10);
    }

private:
    void commandCallback(const example_interfaces::msg::String::SharedPtr msg)
    {
        // Send the command to the serial port
        std::string led_command;
        std::cout<<"Enter LED command: ";
        std::cin >>led_command;
        std::string command = msg->data + "\n";
        serial_port_.write(led_command);
    }
        rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr command_subscriber_;
        serial::Serial serial_port_;
};

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    // auto shared a pointer had a node
    auto node = std::make_shared<SensorReader>();
    rclcpp::spin(node);

    rclcpp::shutdown(); // stop share node
    return 0;
}