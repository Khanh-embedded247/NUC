/***************************************************************************/ /**
 * \file sim_time.cpp
 *
 * \brief Node that publishes simulated time to the /clock topic
 *
 * \author Paul Bouchier
 * \date January 27, 2016
 *
 * \section license License (BSD-3)
 
 ******************************************************************************/

#include "rclcpp/rclcpp.hpp"
#include <rosgraph_msgs/msg/clock.hpp>


#include <sys/time.h>

#define SIM_TIME_INCREMENT_US 10000

/*
 * Nút này xuất bản các khoảng thời gian tăng thêm 1ms cho chủ đề/đồng hồ. Nó làm như vậy
 * với tốc độ được xác định bởi sim_speedup (hệ số tăng tốc mô phỏng), điều này sẽ
 * được thông qua
 * dưới dạng tham số riêng tư.
 */

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto sim_time_node = rclcpp::Node::make_shared("sim_time_source");

  // hỗ trợ bội số tích phân của thời gian đồng hồ treo tường để tăng tốc mô phỏng
  int sim_speedup;  // yếu tố không thể thiếu để tăng tốc độ mô phỏng
  auto node_priv = std::make_shared<rclcpp::Node>("~");
  node_priv->get_parameter_or<int>("sim_speedup", sim_speedup, 1);

  // get the current time & populate sim_time with it
   struct timeval start;
  int rv = gettimeofday(&start, NULL);
  usleep(1000);
  struct timeval now;
  rv = gettimeofday(&now, NULL);
  if (0 != rv)
  {
    RCLCPP_ERROR(sim_time_node->get_logger(), "Invalid return from gettimeofday: %d", rv);
    return -1;
  }

  rosgraph_msgs::msg::Clock sim_time;
  sim_time.clock.sec = now.tv_sec - start.tv_sec;
  sim_time.clock.nanosec = now.tv_usec * 1000;
  auto sim_time_pub = sim_time_node->create_publisher<rosgraph_msgs::msg::Clock>("clock", 1);

  RCLCPP_INFO(sim_time_node->get_logger(), "Starting simulation time publisher at time: %d.%d",
              sim_time.clock.sec, sim_time.clock.nanosec);

  while (rclcpp::ok())
  {
    sim_time_pub->publish(sim_time);

    sim_time.clock.nanosec = sim_time.clock.nanosec + SIM_TIME_INCREMENT_US * 1000;
    while (sim_time.clock.nanosec > 1000000000)
    {
      sim_time.clock.nanosec -= 1000000000;
      ++sim_time.clock.sec;
    }

    usleep(SIM_TIME_INCREMENT_US / sim_speedup);
    rclcpp::spin_some(sim_time_node);
  }

  return 0;
}