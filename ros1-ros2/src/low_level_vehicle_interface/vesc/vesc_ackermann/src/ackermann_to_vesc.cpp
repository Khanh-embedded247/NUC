// -*- mode:c++; fill-column: 100; -*-
//chuyển đổi lệnh điều khiển từ dạng lái xe Ackermann sang các tín hiệu điều khiển cần thiết
// cho mô-đun điều khiển xe điện VESC (Vedder Electronic Speed Controller)
#include "vesc_ackermann/ackermann_to_vesc.hpp"

#include <cmath>
#include <sstream>
#include<rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include<vesc_ackermann/ackermann_to_vesc.hpp>

namespace vesc_ackermann
{

template <typename T>
inline bool getRequiredParam(const rclcpp::Node::SharedPtr& nh, std::string name, T& value);

AckermannToVesc::AckermannToVesc(const rclcpp::Node::SharedPtr& nh, const rclcpp::Node::SharedPtr& private_nh)
  : Node("ackermann_to_vesc_node")
{
  // lấy thông số chuyển đổi
  if (!getRequiredParam(nh, "speed_to_erpm_gain", speed_to_erpm_gain_))//tăng tốc độ
    return;
  if (!getRequiredParam(nh, "speed_to_erpm_offset", speed_to_erpm_offset_))//bù tốc độ
    return;
  if (!getRequiredParam(nh, "steering_angle_to_servo_gain", steering_to_servo_gain_))//góc lái 
    return;
  if (!getRequiredParam(nh, "steering_angle_to_servo_offset", steering_to_servo_offset_))//goc lái bù servo
    return;

  //tạo các nhà xuất bản cho các lệnh vesc electric-RPM (tốc độ) và servo
  erpm_pub_ = this->create_publisher<std_msgs::msg::Float64>("commands/motor/speed", 10);
  servo_pub_ = this->create_publisher<std_msgs::msg::Float64>("commands/servo/position", 10);

  // đăng ký chủ đề ackermann
  ackermann_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
    "ackermann_cmd", 10, std::bind(&AckermannToVesc::ackermannCmdCallback, this, std::placeholders::_1));
}

void AckermannToVesc::ackermannCmdCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr cmd)
{
  // calc vesc electric RPM (speed)
  auto erpm_msg = std::make_unique<std_msgs::msg::Float64>();
  erpm_msg->data = speed_to_erpm_gain_ * cmd->drive.speed + speed_to_erpm_offset_;

  // calc steering angle (servo)
  auto servo_msg = std::make_unique<std_msgs::msg::Float64>();
  servo_msg->data = steering_to_servo_gain_ * cmd->drive.steering_angle + steering_to_servo_offset_;

  // publish
  if (rclcpp::ok()) {
    erpm_pub_->publish(std::move(erpm_msg));
    servo_pub_->publish(std::move(servo_msg));
  }
}

template <typename T>
inline bool getRequiredParam(const rclcpp::Node::SharedPtr& nh, std::string name, T& value)
{
  if (nh->get_parameter(name, value))
    return true;

  RCLCPP_FATAL(nh->get_logger(), "AckermannToVesc: Parameter %s is required.", name.c_str());
  return false;
}

} // namespace vesc_ackermann
