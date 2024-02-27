#include <memory>
#include <rclcpp/rclcpp.hpp>
// #include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "vesc_ackermann/vesc_to_odom.hpp"

namespace vesc_ackermann
{

class VescToOdomNodelet : public rclcpp::Node, public nodelet::Nodelet
{
public:
  VescToOdomNodelet() {}

private:
  void onInit() override;

  std::shared_ptr<VescToOdom> vesc_to_odom_;
}; // class VescToOdomNodelet

void VescToOdomNodelet::onInit()
{
  NODELET_DEBUG("Initializing RACECAR VESC odometry estimator nodelet");
  vesc_to_odom_ = std::make_shared<VescToOdom>(get_node_base_interface(), get_node_logging_interface(), get_node_clock_interface(), get_node_parameters_interface());
}

} // namespace vesc_ackermann

PLUGINLIB_EXPORT_CLASS(vesc_ackermann::VescToOdomNodelet, nodelet::Nodelet);
