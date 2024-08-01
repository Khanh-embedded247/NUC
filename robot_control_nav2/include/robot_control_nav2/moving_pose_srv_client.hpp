#ifndef MOVING_POSE_SRV_CLIENT_HPP_
#define MOVING_POSE_SRV_CLIENT_HPP_


#include <memory>
#include <chrono>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>

#include <yaml-cpp/yaml.h>

#include "robot_msgs/srv/operator_two_ints.hpp"
#include "std_msgs/msg/int32.hpp"
class MovingPoseSrvClient: public rclcpp::Node
{
public: 
    MovingPoseSrvClient();
private:

    std::thread _thread;
    rclcpp::TimerBase::SharedPtr _timer;
 
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr _response_sub;
    void response_callback(const std_msgs::msg::Int32 &msg);
 
    rclcpp::Client<robot_msgs::srv::OperatorTwoInts>::SharedPtr _client;
    
    void operator_two_ints_request(std_msgs::msg::Int32 a, std_msgs::msg::Int32 b);
    void timer_callback();
    void call_operator_two_ints(std_msgs::msg::Int32 a, std_msgs::msg::Int32 b);

    void excute_main();

};


#endif  
