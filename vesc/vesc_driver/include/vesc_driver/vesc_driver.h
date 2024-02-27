/*********************************************************************

// TODO: Migrate to ROS2
*/
#ifndef VESC_DRIVER_VESC_DRIVER_H_
#define VESC_DRIVER_VESC_DRIVER_H_

#include <cassert>
#include <cmath>
#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include <boost/optional.hpp>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/float64.hpp>
#include </home/robotic/vesc/vesc_msgs/msg/vesc_state_stamped.hpp>

#include "/home/robotic/vesc/vesc_driver/include/vesc_driver/vesc_interface.h"
#include "/home/robotic/vesc/vesc_driver/include/vesc_driver/vesc_packet.h"

namespace vesc_driver
{
class VescDriver : public rclcpp::Node
{
public:
  explicit VescDriver(const rclcpp::NodeOptions & options);
  VescDriver(
        std::shared_ptr<rclcpp::Node> node,
        std::shared_ptr<rclcpp::Node> private_node
    );
private:
  // interface to the VESC
  VescInterface vesc_;
  void vescPacketCallback(const std::shared_ptr<VescPacket const>& packet);
  void vescErrorCallback(const std::string& error);

  //Định nghĩa cấu trúc thể hiện các giới hạn trên các lệnh VESC
  struct CommandLimit
  {
    CommandLimit(const rclcpp::Node::SharedPtr& node, const std::string& str,
                 const boost::optional<double>& min_lower = boost::optional<double>(),
                 const boost::optional<double>& max_upper = boost::optional<double>());
    double clip(double value);
    std::string name;
    boost::optional<double> lower;
    boost::optional<double> upper;
  };
  //Các node
  CommandLimit duty_cycle_limit_;
  CommandLimit current_limit_;
  CommandLimit brake_limit_;
  CommandLimit speed_limit_;
  CommandLimit position_limit_;
  CommandLimit servo_limit_;

  // ROS services
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr state_pub_;//trạng thái
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr servo_sensor_pub_;//cảm biến đc
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr duty_cycle_sub_;//độ rộng xung
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr current_sub_;//điện áp 
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr brake_sub_;//phanh
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_sub_;//tốc độ
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr position_sub_;//vị trí
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr servo_sub_;//động cơ
  rclcpp::TimerBase::SharedPtr timer_;

  // Chế độ lái
  typedef enum
  {
    MODE_INITIALIZING,//khởi tạo chế độ
    MODE_OPERATING//tchês độ hđ
  } driver_mode_t;

  // other variables
  driver_mode_t driver_mode_;  ///chế độ người lái máy
  int fw_version_major_;       ///< phiên bản chính của firmware được báo cáo bởi vesc
  int fw_version_minor_;       ///< phiên bản phần sụn nhỏ được báo cáo bởi vesc
  int num_motor_pole_pairs_;   //số cặp cực động cơ

  // ROS callbacks
  void timerCallback();
  void dutyCycleCallback(const std_msgs::msg::Float64::SharedPtr duty_cycle);
  void currentCallback(const std_msgs::msg::Float64::SharedPtr current);
  void brakeCallback(const std_msgs::msg::Float64::SharedPtr brake);
  void speedCallback(const std_msgs::msg::Float64::SharedPtr speed);
  void positionCallback(const std_msgs::msg::Float64::SharedPtr position);
  void servoCallback(const std_msgs::msg::Float64::SharedPtr servo);
};

}  // namespace vesc_driver

#endif  // VESC_DRIVER_VESC_DRIVER_H_

