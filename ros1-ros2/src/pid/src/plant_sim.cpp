#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

namespace plant_sim
{
  // Global so it can be passed from the callback fxn to main
  static double control_effort = 0.0;
  static bool reverse_acting = false;
    // Declare the variables
  double temp = 0.0;
  double speed = 0.0;
  double displacement = 0.0;
  double control_effort = 0.0;
}  // namespace plant_sim

using namespace plant_sim;

// Callback when something is published on 'control_effort'
void controlEffortCallback(const std_msgs::msg::Float64::SharedPtr control_effort_input)
{
  // the stabilizing control effort
  if (reverse_acting)
  {
    control_effort = -control_effort_input->data;
  }
  else
  {
    control_effort = control_effort_input->data;
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto sim_node = std::make_shared<rclcpp::Node>("plant");

  while (rclcpp::ok() && rclcpp::Time(0) == sim_node->now())
  {
    RCLCPP_INFO(sim_node->get_logger(), "Plant_sim spinning waiting for time to become non-zero");
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  auto node_priv = std::make_shared<rclcpp::Node>("~");
  int plant_order = 1;
  bool reverse_acting = false;
  node_priv->declare_parameter("plant_order", 1);
  node_priv->declare_parameter("reverse_acting", false);
  node_priv->get_parameter("plant_order", plant_order);
  node_priv->get_parameter("reverse_acting", reverse_acting);

  if (plant_order == 1)
  {
    RCLCPP_INFO(sim_node->get_logger(), "Starting simulation of a first-order plant.");
  }
  else if (plant_order == 2)
  {
    RCLCPP_INFO(sim_node->get_logger(), "Starting simulation of a second-order plant.");
  }
  else
  {
    RCLCPP_ERROR(sim_node->get_logger(), "Error: Invalid plant type parameter, must be 1 or 2");
    return -1;
  }

  // Advertise a plant state msg
  auto servo_state_pub = sim_node->create_publisher<std_msgs::msg::Float64>("state", 1);

  // Subscribe to "control_effort" topic to get a controller_msg.msg
  auto sub = sim_node->create_subscription<std_msgs::msg::Float64>("control_effort", 1, controlEffortCallback);

  int loop_counter = 0;
  double delta_t = 0.01;
  rclcpp::Rate loop_rate(1 / delta_t);  // Control rate in Hz

  // Initialize 1st-order (e.g temp controller) process variables
  double temp_rate = 0;  // rate of temp change

  // Initialize 2nd-order (e.g. servo-motor with load) process variables
  double speed = 0;         // meters/sec
  double acceleration = 0;  // meters/sec^2
  double mass = 0.1;        // in kg
  double friction = 1.0;    // a decelerating force factor
  double stiction = 1;      // control_effort must exceed this before stationary servo moves
  double Kv = 1;            // motor constant: force (newtons) / volt
  double Kbackemf = 0;      // Volts of back-emf per meter/sec of speed
  double decel_force;       // decelerating force

  while (rclcpp::ok())
  {
    rclcpp::spin_some(sim_node);

    switch (plant_order)
    {
      case 1:  // First order plant
        temp_rate = (0.1 * temp) + control_effort;
        temp = temp + temp_rate * delta_t;

        servo_state_pub->publish(std_msgs::msg::Float64().set_data(temp));
        break;

      case 2:  // Second order plant
        if (fabs(speed) < 0.001)
        {
          // if nearly stopped, stop it & require overcoming stiction to restart
          speed = 0;
          if (fabs(control_effort) < stiction)
          {
            control_effort = 0;
          }
        }

        // Update the servo.
        // control_effort: the voltage applied to the servo. Output from PID
        // controller. It is
        //   opposed by back emf (expressed as speed) to produce a net force.
        //   Range: -1 to +1
        // displacement: the actual value of the servo output position. Input to
        // PID controller

        decel_force = -(speed * friction);  // can be +ve or -ve. Linear with speed
        acceleration = ((Kv * (control_effort - (Kbackemf * speed)) + decel_force) / mass);  // a = F/m
        speed = speed + (acceleration * delta_t);
        displacement = displacement + speed * delta_t;

        servo_state_pub->publish(std_msgs::msg::Float64().set_data(displacement));
        break;

      default:
        RCLCPP_ERROR(sim_node->get_logger(), "Invalid plant_order: %d", plant_order);
        return (-1);
    }

    loop_rate.sleep();
  }

  return 0;
}
