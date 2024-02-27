/***************************************************************************/ /**
 * \file controller.cpp
 *
 * \brief Simple PID controller with dynamic reconfigure
 * \author Andy Zelenak
 * \date March 8, 2015
 *
 * \section license License (BSD-3)
 
 ******************************************************************************/

// Subscribe to a topic about the state of a dynamic system and calculate
// feedback to
// stabilize it.

#include "rclcpp/rclcpp.hpp"
#include "pid/pid.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("controller");

  pid_ns::PidObject my_pid(node);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}

