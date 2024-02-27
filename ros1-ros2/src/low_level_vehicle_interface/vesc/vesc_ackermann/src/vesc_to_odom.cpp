// thực hiện chuyển đổi dữ liệu từ bộ điều khiển xe điện thành dữ liệu odometry và
// có thể broadcast transform nếu được cấu hình
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "vesc_ackermann/vesc_to_odom.hpp"

namespace vesc_ackermann
{

template <typename T>
inline bool getRequiredParam(const rclcpp::Node::SharedPtr& nh, std::string name, T& value);

VescToOdom::VescToOdom(const rclcpp::Node::SharedPtr& nh, const rclcpp::Node::SharedPtr& private_nh) :
  Node("vesc_to_odom_node"), odom_frame_("odom"), base_frame_("base_link"),
  use_servo_cmd_(true), publish_tf_(false), x_(0.0), y_(0.0), yaw_(0.0)
{
  // get ROS parameters
  private_nh->get_parameter("odom_frame", odom_frame_);
  private_nh->get_parameter("base_frame", base_frame_);
  private_nh->get_parameter("use_servo_cmd_to_calc_angular_velocity", use_servo_cmd_);
  //speed_to_erpm_gain_,speed_to_erpm_offset_,steering_to_servo_gain_,steering_to_servo_offset_,wheelbase_
  //Các tham số để chuyển đổi dữ liệu từ bộ điều khiển.
  if (!getRequiredParam(nh, "speed_to_erpm_gain", speed_to_erpm_gain_))
    return;
  if (!getRequiredParam(nh, "speed_to_erpm_offset", speed_to_erpm_offset_))
    return;
  if (use_servo_cmd_) {
    if (!getRequiredParam(nh, "steering_angle_to_servo_gain", steering_to_servo_gain_))
      return;
    if (!getRequiredParam(nh, "steering_angle_to_servo_offset", steering_to_servo_offset_))
      return;
    if (!getRequiredParam(nh, "wheelbase", wheelbase_))
      return;
  }
  private_nh->get_parameter("publish_tf", publish_tf_);

  // create odom publisher
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

  // create tf broadcaster
  if (publish_tf_) {
    tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

  // đăng ký trạng thái vesc và, tùy chọn, lệnh servo
  vesc_state_sub_ = create_subscription<vesc_msgs::msg::VescStateStamped>(
    "sensors/core", 10, std::bind(&VescToOdom::vescStateCallback, this, std::placeholders::_1));
  if (use_servo_cmd_) {
    servo_sub_ = create_subscription<std_msgs::msg::Float64>(
      "sensors/servo_position_command", 10, std::bind(&VescToOdom::servoCmdCallback, this, std::placeholders::_1));
  }
}

void VescToOdom::vescStateCallback(const vesc_msgs::msg::VescStateStamped::SharedPtr state)
{
  //kiểm tra xem chúng ta có lệnh servo cuối cùng hay không nếu chúng ta phụ thuộc vào nó để biết vận tốc góc
  if (use_servo_cmd_ && !last_servo_cmd_)
    return;

  // chuyển đổi sang các đơn vị kỹ thuật
  double current_speed = (state->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;
  double current_steering_angle(0.0), current_angular_velocity(0.0);
  if (use_servo_cmd_) {
    current_steering_angle =
      (last_servo_cmd_->data - steering_to_servo_offset_) / steering_to_servo_gain_;
    current_angular_velocity = current_speed * tan(current_steering_angle) / wheelbase_;
  }

  // sử dụng trạng thái hiện tại làm trạng thái cuối cùng sau lần gần nhất
  if (!last_state_)
    last_state_ = state;

  // tính thời gian đã trôi qua
  rclcpp::Duration dt = state->header.stamp - last_state_->header.stamp;

  /** @todo could probably do better propagating odometry, e.g. trapezoidal integration */

  // truyền phép đo hình
  double x_dot = current_speed * cos(yaw_);
  double y_dot = current_speed * sin(yaw_);
  x_ += x_dot * dt.seconds();
  y_ += y_dot * dt.seconds();
  if (use_servo_cmd_)
    yaw_ += current_angular_velocity * dt.seconds();

  // save state for next time
  last_state_ = state;

  // publish odometry message
  auto odom = std::make_unique<nav_msgs::msg::Odometry>();
  odom->header.frame_id = odom_frame_;
  odom->header.stamp = state->header.stamp;
  odom->child_frame_id = base_frame_;

  // Position
  odom->pose.pose.position.x = x_;
  odom->pose.pose.position.y = y_;
  odom->pose.pose.orientation.x = 0.0;
  odom->pose.pose.orientation.y = 0.0;
  odom->pose.pose.orientation.z = sin(yaw_ / 2.0);
  odom->pose.pose.orientation.w = cos(yaw_ / 2.0);

  // vị trí giả định có sai số (k chắc về định vị đúng vị trí )
  /** @todo Think about position uncertainty, perhaps get from parameters? */
  odom->pose.covariance[0] = 0.2; ///< x
  odom->pose.covariance[7] = 0.2; ///< y
  odom->pose.covariance[35] = 0.4; ///< yaw

  // Vận tốc ("trong khung tọa độ được cung cấp bởi child_frame_id")
  odom->twist.twist.linear.x = current_speed;
  odom->twist.twist.linear.y = 0.0;
  odom->twist.twist.angular.z = current_angular_velocity;

  // Sự không chắc chắn về vận tốc(vận tốc từ vị trí thay đổi )
  /** @todo Think about velocity uncertainty */

  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.header.stamp = now();
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, yaw_);
    tf.transform.rotation.x = quat.x();
    tf.transform.rotation.y = quat.y();
    tf.transform.rotation.z = quat.z();
    tf.transform.rotation.w = quat.w();

    tf_pub_->sendTransform(tf);
  }

  if (rclcpp::ok()) {
    odom_pub_->publish(std::move(odom));
  }
}

void VescToOdom::servoCmdCallback(const std_msgs::msg::Float64::SharedPtr servo)
{
  last_servo_cmd_ = servo;
}

template <typename T>
inline bool getRequiredParam(const rclcpp::Node::SharedPtr& nh, std::string name, T& value)
{
  if (nh->get_parameter(name, value))
    return true;

  RCLCPP_FATAL(nh->get_logger(), "VescToOdom: Parameter %s is required.", name.c_str());
  return false;
}

} // namespace vesc_ackermann
