#include <rclcpp/rclcpp.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>

/* một node ROS đơn giản được thiết kế để chuyển đổi dữ liệu giữa hai bánh xe và hai động cơ */
// #define RR_MOTOR_CALIB  1.1400
// #define SIDEWAY_CALIB   1.1000

#define RR_MOTOR_CALIB  1//giá trị hiệu chỉnh cho động cơ bánh sau 
#define SIDEWAY_CALIB   1//giá trị hiệu chỉnh cho động cơ đẩy bên 

/* Convert from velocity command to duty_cycle
 * speed (feedback) 600.0
 * real rpm = 40
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
#define VEL_TO_DUTY     1

// encoder front left
rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_enc_l;
rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_speed_l;
rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr sub_enc_l;

// encoder front right
rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_enc_r;
rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_speed_r;
rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr sub_enc_r;

/* ------------------------------- Motor-----------------------------------*/
// motor front left
rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_motor_l;
rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_motor_l;

// motor front right;
rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_motor_r;
rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_motor_r;

/* ------------------------------ Message ------------------------------------*/
std_msgs::msg::Int64 encoder_l;
std_msgs::msg::Int64 encoder_r;

std_msgs::msg::Float64 duty_cycle_l;
std_msgs::msg::Float64 duty_cycle_r;

std_msgs::msg::Float64 speed_l;
std_msgs::msg::Float64 speed_r;

/* ------------------------------ Encoder callback ------------------------------------*/
void callback_enc_l(const vesc_msgs::msg::VescStateStamped::SharedPtr msg)
{
    speed_l.data = msg->state.speed;
    encoder_l.data = static_cast<int>(msg->state.distance_traveled);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Data left sent");
}

void callback_enc_r(const vesc_msgs::msg::VescStateStamped::SharedPtr msg)
{
    speed_r.data = msg->state.speed;
    encoder_r.data = static_cast<int>(msg->state.distance_traveled);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Data right sent");
}

/* ------------------------------ Motor callback ------------------------------------*/
void callback_motor_l(const std_msgs::msg::Float64::SharedPtr msg)
{
    duty_cycle_l.data = msg->data * VEL_TO_DUTY;
    // duty_cycle_l.data = 0.2;
}

void callback_motor_r(const std_msgs::msg::Float64::SharedPtr msg)
{ 
    duty_cycle_r.data = msg->data * VEL_TO_DUTY * SIDEWAY_CALIB;
    // duty_cycle_r.data = 0.2;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("vesc_converter");

    rclcpp::Rate rate(1000); // ROS Rate at 1000Hz

    std_msgs::msg::Float64 starting_current;
    starting_current.data = 3;

    std_msgs::msg::Float64 braking_current;
    braking_current.data = 3;

    std_msgs::msg::Float64 normal_current;
    normal_current.data = 1;

    // Encoders
    pub_enc_l = node->create_publisher<std_msgs::msg::Int64>("/lwheel", 10);
    pub_enc_r = node->create_publisher<std_msgs::msg::Int64>("/rwheel", 10);

    sub_enc_l = node->create_subscription<vesc_msgs::msg::VescStateStamped>("/left/sensors/core", 10, callback_enc_l);
    sub_enc_r = node->create_subscription<vesc_msgs::msg::VescStateStamped>("/right/sensors/core", 10, callback_enc_r);

    pub_speed_l = node->create_publisher<std_msgs::msg::Float64>("/lspeed", 10); // rad/sec
    pub_speed_r = node->create_publisher<std_msgs::msg::Float64>("/rspeed", 10); // rad/sec

    // Motors
    pub_motor_l = node->create_publisher<std_msgs::msg::Float64>("/left/commands/motor/duty_cycle", 10);
    pub_motor_r = node->create_publisher<std_msgs::msg::Float64>("/right/commands/motor/duty_cycle", 10);

    sub_motor_l = node->create_subscription<std_msgs::msg::Float64>("/lwheel_pid", 10, callback_motor_l);
    sub_motor_r = node->create_subscription<std_msgs::msg::Float64>("/rwheel_pid", 10, callback_motor_r);

    while (rclcpp::ok())
    {
        pub_enc_l->publish(encoder_l);
        pub_enc_r->publish(encoder_r);

        pub_speed_l->publish(speed_l);
        pub_speed_r->publish(speed_r);

        pub_motor_l->publish(duty_cycle_l);
        pub_motor_r->publish(duty_cycle_r);

        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}
