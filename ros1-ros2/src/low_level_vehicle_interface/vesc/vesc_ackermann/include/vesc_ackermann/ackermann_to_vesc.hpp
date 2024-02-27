// -*- mode:c++; fill-column: 100; -*-
/*một node ROS, được thiết kế để chuyển đổi lệnh điều khiển từ dạng "Ackermann drive" sang dạng hiểu được bởi VESC */
#ifndef VESC_ACKERMANN_ACKERMANN_TO_VESC_HPP_
#define VESC_ACKERMANN_ACKERMANN_TO_VESC_HPP_

#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

namespace vesc_ackermann
{

class AckermannToVesc : public rclcpp::Node
{
public:

  AckermannToVesc(const rclcpp::NodeOptions & options);

private:
  // ROS parameters
  // conversion gain and offset
  double speed_to_erpm_gain_, speed_to_erpm_offset_;
  double steering_to_servo_gain_, steering_to_servo_offset_;

  /** @todo consider also providing an interpolated look-up table conversion */

  // ROS publishers and subscribers
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr erpm_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr servo_pub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_sub_;

  // ROS callbacks
  void ackermannCmdCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr cmd);
};

} // namespace vesc_ackermann

#endif // VESC_ACKERMANN_ACKERMANN_TO_VESC_HPP_
