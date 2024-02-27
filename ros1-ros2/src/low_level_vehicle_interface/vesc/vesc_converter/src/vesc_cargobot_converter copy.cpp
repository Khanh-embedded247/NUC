
/* Calib wheel because they dont run at the same speed when receiving the same duty_cycle */
#define RR_MOTOR_CALIB  1.1400
#define SIDEWAY_CALIB   1.1000

/* Convert from velocity command to duty_cycle
 * speed (feedback) 600.0
 * real rpm = 40(v/p))
 * duty_cycle (command) = 0.1
 * speed = 0.17 * 3.14 * rpm / 60 = 0.0089 * rpm
 * duty_cycle (command) = 0.1 ~ 0.3556 m/s
 * --> duty_cycle = vel * 0.281 
 */

 /* Convert from velocity command to duty_cycle
 * speed (feedback) 
 * real rpm = 9
 * duty_cycle (command) = 0.1
 * speed = 0.17 * 3.14 * rpm / 60 = 0.0089 * rpm
 * duty_cycle (command) = 0.1 ~ 0.08 m/s
 * --> duty_cycle = vel * 0.35
 */

  /* Convert from velocity command to duty_cycle
 * speed (feedback) 
 * real rpm = 12
 * duty_cycle (command) = 0.1
 * speed = 0.17 * 3.14 * rpm / 60 = 0.0089 * rpm
 * duty_cycle (command) = 0.1 ~ 0.1 m/s
 * --> duty_cycle = vel * 0.35
 */

 /*
    1 round : 9s
 */

#include <rclcpp/rclcpp.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>

#define RR_MOTOR_CALIB 1.1400
#define SIDEWAY_CALIB 1.1000
#define VEL_TO_DUTY 0.35

class VescConverter : public rclcpp::Node
{
public:
  VescConverter() : Node("vesc_converter")
  {
    // Encoders
    pub_flen = create_publisher<std_msgs::msg::Int64>("/flwheel", 10);
    pub_fren = create_publisher<std_msgs::msg::Int64>("/frwheel", 10);
    pub_rlen = create_publisher<std_msgs::msg::Int64>("/rlwheel", 10);
    pub_rren = create_publisher<std_msgs::msg::Int64>("/rrwheel", 10);

    sub_flen = create_subscription<vesc_msgs::msg::VescStateStamped>(
        "/front_left/sensors/core", 10, std::bind(&VescConverter::callback_flen, this, std::placeholders::_1));
    sub_fren = create_subscription<vesc_msgs::msg::VescStateStamped>(
        "/front_right/sensors/core", 10, std::bind(&VescConverter::callback_fren, this, std::placeholders::_1));
    sub_rlen = create_subscription<vesc_msgs::msg::VescStateStamped>(
        "/rear_left/sensors/core", 10, std::bind(&VescConverter::callback_rlen, this, std::placeholders::_1));
    sub_rren = create_subscription<vesc_msgs::msg::VescStateStamped>(
        "/rear_right/sensors/core", 10, std::bind(&VescConverter::callback_rren, this, std::placeholders::_1));

    // Motors
    pub_flmotor = create_publisher<std_msgs::msg::Float64>("/front_left/commands/motor/duty_cycle", 10);
    pub_frmotor = create_publisher<std_msgs::msg::Float64>("/front_right/commands/motor/duty_cycle", 10);
    pub_rlmotor = create_publisher<std_msgs::msg::Float64>("/rear_left/commands/motor/duty_cycle", 10);
    pub_rrmotor = create_publisher<std_msgs::msg::Float64>("/rear_right/commands/motor/duty_cycle", 10);

    sub_flmotor = create_subscription<std_msgs::msg::Float32>(
        "/flwheel_vtarget", 10, std::bind(&VescConverter::callback_flmotor, this, std::placeholders::_1));
    sub_frmotor = create_subscription<std_msgs::msg::Float32>(
        "/frwheel_vtarget", 10, std::bind(&VescConverter::callback_frmotor, this, std::placeholders::_1));
    sub_rlmotor = create_subscription<std_msgs::msg::Float32>(
        "/rlwheel_vtarget", 10, std::bind(&VescConverter::callback_rlmotor, this, std::placeholders::_1));
    sub_rrmotor = create_subscription<std_msgs::msg::Float32>(
        "/rrwheel_vtarget", 10, std::bind(&VescConverter::callback_rrmotor, this, std::placeholders::_1));

    
  }

private:
  // Callbacks
  void callback_flen(const vesc_msgs::msg::VescStateStamped::SharedPtr msg)
  {
    speed_fl.data = msg->state.speed;
    encoder_fl.data = static_cast<int>(msg->state.distance_traveled);
    RCLCPP_INFO(get_logger(), "Data front left sent");
  }

  void callback_fren(const vesc_msgs::msg::VescStateStamped::SharedPtr msg)
  {
    speed_fr.data = msg->state.speed;
    encoder_fr.data = static_cast<int>(msg->state.distance_traveled);
    RCLCPP_INFO(get_logger(), "Data front right sent");
  }

  void callback_rlen(const vesc_msgs::msg::VescStateStamped::SharedPtr msg)
  {
    speed_rl.data = msg->state.speed;
    encoder_rl.data = static_cast<int>(msg->state.distance_traveled);
    RCLCPP_INFO(get_logger(), "Data rear left sent");
  }

  void callback_rren(const vesc_msgs::msg::VescStateStamped::SharedPtr msg)
  {
    speed_rr.data = msg->state.speed;
    encoder_rr.data = static_cast<int>(msg->state.distance_traveled);
    RCLCPP_INFO(get_logger(), "Data rear right sent");
  }

  void callback_flmotor(const std_msgs::msg::Float32::SharedPtr msg)
  {
    duty_cycle_fl.data = msg->data * VEL_TO_DUTY;
  }

  void callback_frmotor(const std_msgs::msg::Float32::SharedPtr msg)
  {
    duty_cycle_fr.data = msg->data * VEL_TO_DUTY * SIDEWAY_CALIB;
  }

  void callback_rlmotor(const std_msgs::msg::Float32::SharedPtr msg)
  {
    duty_cycle_rl.data = msg->data * VEL_TO_DUTY;
  }

  void callback_rrmotor(const std_msgs::msg::Float32::SharedPtr msg)
  {
    duty_cycle_rr.data = msg->data * VEL_TO_DUTY * SIDEWAY_CALIB * RR_MOTOR_CALIB;
  }

  // Publishers and Subscribers
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_flen;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_fren;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_rlen;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_rren;

  rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr sub_flen;
  rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr sub_fren;
  rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr sub_rlen;
  rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr sub_rren;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_flmotor;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_frmotor;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_rlmotor;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_rrmotor;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_flmotor;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_frmotor;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_rlmotor;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_rrmotor;

  // Messages
  std_msgs::msg::Int64 encoder_fl;
  std_msgs::msg::Int64 encoder_fr;
  std_msgs::msg::Int64 encoder_rl;
  std_msgs::msg::Int64 encoder_rr;

  std_msgs::msg::Float64 duty_cycle_fl;
  std_msgs::msg::Float64 duty_cycle_fr;
  std_msgs::msg::Float64 duty_cycle_rl;
  std_msgs::msg::Float64 duty_cycle_rr;

  std_msgs::msg::Float64 speed_fl;
  std_msgs::msg::Float64 speed_fr;
  std_msgs::msg::Float64 speed_rl;
  std_msgs::msg::Float64 speed_rr;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VescConverter>());
  rclcpp::shutdown();

  return 0;
}
