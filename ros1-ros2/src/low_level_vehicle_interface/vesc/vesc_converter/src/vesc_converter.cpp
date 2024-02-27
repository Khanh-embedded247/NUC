#include <rclcpp/rclcpp.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>

class VescConverter : public rclcpp::Node
{
public:
  VescConverter() : Node("vesc_converter")
  {
    pub1 = create_publisher<std_msgs::msg::Int64>("/lwheel", 10);
    pub2 = create_publisher<std_msgs::msg::Int64>("/rwheel", 10);
    pub3 = create_publisher<std_msgs::msg::Float64>("/left/commands/motor/duty_cycle", 10);
    pub4 = create_publisher<std_msgs::msg::Float64>("/right/commands/motor/duty_cycle", 10);

    sub1 = create_subscription<vesc_msgs::msg::VescStateStamped>(
        "/left/sensors/core", 10, std::bind(&VescConverter::callback1, this, std::placeholders::_1));
    sub2 = create_subscription<vesc_msgs::msg::VescStateStamped>(
        "/right/sensors/core", 10, std::bind(&VescConverter::callback2, this, std::placeholders::_1));
    sub3 = create_subscription<std_msgs::msg::Float32>(
        "/lmotor_cmd", 10, std::bind(&VescConverter::callback3, this, std::placeholders::_1));
    sub4 = create_subscription<std_msgs::msg::Float32>(
        "/rmotor_cmd", 10, std::bind(&VescConverter::callback4, this, std::placeholders::_1));

    // Initialize your variables here if needed

    // Use timers for periodic tasks
    timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&VescConverter::publishData, this));
  }

private:
  // Callbacks
  void callback1(const vesc_msgs::msg::VescStateStamped::SharedPtr msg)
  {
    // Update variables based on received data
    speed1.data = msg->state.speed;
    encoder1.data = static_cast<int>(msg->state.distance_traveled);
  }

  void callback2(const vesc_msgs::msg::VescStateStamped::SharedPtr msg)
  {
    // Update variables based on received data
    speed2.data = msg->state.speed;
    encoder2.data = static_cast<int>(msg->state.distance_traveled);
  }

  void callback3(const std_msgs::msg::Float32::SharedPtr msg)
  {
    // Update variables based on received data
    duty_cycle1.data = msg->data;
  }

  void callback4(const std_msgs::msg::Float32::SharedPtr msg)
  {
    // Update variables based on received data
    duty_cycle2.data = msg->data;
  }

  void publishData()
  {
    // Publish data based on your logic
    pub1->publish(encoder1);
    pub2->publish(encoder2);
    pub3->publish(duty_cycle1);
    pub4->publish(duty_cycle2);
  }

  // Publishers and Subscribers
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub1;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub2;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub3;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub4;

  rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr sub1;
  rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr sub2;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub3;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub4;

  // Timers
  rclcpp::TimerBase::SharedPtr timer_;

  // Messages
  std_msgs::msg::Int64 encoder1;
  std_msgs::msg::Int64 encoder2;
  std_msgs::msg::Float64 duty_cycle1;
  std_msgs::msg::Float64 duty_cycle2;
  std_msgs::msg::Float64 speed1;
  std_msgs::msg::Float64 speed2;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VescConverter>());
  rclcpp::shutdown();

  return 0;
}
