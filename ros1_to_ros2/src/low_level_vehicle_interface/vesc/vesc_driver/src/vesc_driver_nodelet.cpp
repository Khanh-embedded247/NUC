/*********************************************************************
 Mục đích của file :
Triển khai một nodelet::Nodelet được thiết kế để cung cấp cách chạy nhiều thuật toán trên một máy, trong một quy trình duy nhất mà không phát sinh chi phí sao chép khi truyền tin nhắn trong nội bộ quy trình
Trong hàm onInit(), tạo một đối tượng VescDriver với các Node chính và Node riêng tư tương ứng với nodelet.
 ********************************************************************/


// #include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include "vesc_driver/vesc_driver.h"

namespace vesc_driver
{
class VescDriverNodelet : public nodelet::Nodelet
{
public:
  VescDriverNodelet()
  {
  }

private:
  virtual void onInit(void);

  std::shared_ptr<VescDriver> vesc_driver_;
};  // class VescDriverNodelet

void VescDriverNodelet::onInit()
{
  RCLCPP_DEBUG(get_logger(), "Initializing VESC driver nodelet");
  vesc_driver_ = std::make_shared<VescDriver>(get_node_base_interface(), get_node_topics_interface());
}

}  // namespace vesc_driver

PLUGINLIB_EXPORT_CLASS(vesc_driver::VescDriverNodelet, nodelet::Nodelet);
