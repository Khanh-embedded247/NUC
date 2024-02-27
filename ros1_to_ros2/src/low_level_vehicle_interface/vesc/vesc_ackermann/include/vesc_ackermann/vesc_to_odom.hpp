// -*- mode:c++; fill-column: 100; -*-
/* một node ROS được thiết kế để chuyển đổi trạng thái từ VESC (Vehicle Electronic Speed Controller) thành thông tin về Odometry (Odom) và
 có khả năng xuất bản thông tin Odometry cũng như broadcast các biến đổi (transform) trong ROS tf*/
#ifndef VESC_ACKERMANN_VESC_TO_ODOM_H_
#define VESC_ACKERMANN_VESC_TO_ODOM_H_

#include <rclcpp/rclcpp.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace vesc_ackermann
{

class VescToOdom : public rclcpp::Node
{
public:

  VescToOdom(const rclcpp::NodeOptions& options);

private:
  // ROS parameters
  std::string odom_frame_;
  std::string base_frame_;
  /** Thông báo trạng thái ,không báo cáo vị trí servo ,nên sử dụng lệnh thay thế*/
  bool use_servo_cmd_;
  // conversion gain and offset
  double speed_to_erpm_gain_, speed_to_erpm_offset_;//: Tham số để chuyển đổi tốc độ thành giá trị ERPM 
  double steering_to_servo_gain_, steering_to_servo_offset_;//Tham số để chuyển đổi góc lái thành giá trị servo hiểu được bởi VESC.
  double wheelbase_;//khoảng cách giữa 2 bánh xe
  bool publish_tf_;

  // trạng thái đo đường
  double x_, y_, yaw_;
  std_msgs::msg::Float64::ConstSharedPtr last_servo_cmd_; //giá trị servo lần cuối nhận được 
  vesc_msgs::msg::VescStateStamped::ConstSharedPtr last_state_; //trạng thái vesc lần cuối nhận được 

  // ROS services
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr odom_pub_;//Publish thônbg tin Odometry
  rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr vesc_state_sub_;//subsribe trạng thái vesc
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr servo_sub_;//subscriber lệnh servo position
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;//Transform broadcaster để broadcast các biến đổi tf.

  // ROS callbacks
  void vescStateCallback(const vesc_msgs::msg::VescStateStamped::SharedPtr state);
  void servoCmdCallback(const std_msgs::msg::Float64::SharedPtr servo);
};

} // namespace vesc_ackermann

#endif // VESC_ACKERMANN_VESC_TO_ODOM_H_
