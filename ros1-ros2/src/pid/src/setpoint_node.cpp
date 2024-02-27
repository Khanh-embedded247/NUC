/***************************************************************************/ /**
 * \file setpoint_node.cpp
 *
 * \brief Node that publishes time-varying setpoint values
 * \author Paul Bouchier
 * \date January 9, 2016
 *
 * \section license License (BSD-3)
 
 ******************************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting setpoint publisher");
  auto setpoint_node = rclcpp::Node::make_shared("setpoint_node");

  while (rclcpp::ok() && rclcpp::Time(0) == setpoint_node->now())
  {
    RCLCPP_INFO(setpoint_node->get_logger(), "Setpoint_node spinning, waiting for time to become non-zero");
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  auto setpoint_pub = setpoint_node->create_publisher<std_msgs::msg::Float64>("setpoint", 1);

  std_msgs::msg::Float64 setpoint;
  setpoint.data = 1.0;
  rclcpp::Rate loop_rate(0.2);  // change setpoint every 5 seconds

  while (rclcpp::ok())
  {
    rclcpp::spin_some(setpoint_node);

    setpoint_pub->publish(setpoint);  // publish twice so graph gets it as a step
    setpoint.data = -setpoint.data;
    setpoint_pub->publish(setpoint);

    loop_rate.sleep();
  }

  return 0;
}

