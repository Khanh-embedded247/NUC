#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <nodelet/nodelet.h>

#include "vesc_ackermann/ackermann_to_vesc.hpp"

namespace vesc_ackermann
{

class AckermannToVescNodelet : public rclcpp::Node
{
public:

  AckermannToVescNodelet(const rclcpp::NodeOptions & options)
    : rclcpp::Node("ackermann_to_vesc_node", options)
  {
    RCLCPP_DEBUG(get_logger(), "Initializing ackermann to VESC nodelet");
    ackermann_to_vesc_ = std::make_shared<AckermannToVesc>(shared_from_this(), get_private_node());
  }

private:
  std::shared_ptr<AckermannToVesc> ackermann_to_vesc_;
};

} // namespace vesc_ackermann

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::AckermannToVescNodelet)
