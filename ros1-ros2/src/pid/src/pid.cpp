
#include "pid/pid.hpp"
#include "rclcpp/parameter_service.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "pid/PidConfig.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "rclcpp/logging.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include<rclcpp/time.hpp>
using namespace pid_ns;
// ror_(3, 0), filtered_error_(3, 0), error_deriv_(3, 0), filtered_error_deriv_(3, 0)
PidObject::PidObject() : error_(3, 0), filtered_error_(3, 0), error_deriv_(3, 0), filtered_error_deriv_(3, 0)
{
  rclcpp::NodeOptions options;
  auto node = std::make_shared<rclcpp::Node>("pid_controller", options);

  while (rclcpp::ok() && rclcpp::Time(0) == rclcpp::Clock().now())
  {
    RCLCPP_INFO(node->get_logger(), "controller spinning, waiting for time to become non-zero");
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  // Get params if specified in launch file or as params on command-line, set defaults
  node->get_parameter_or("Kp", Kp_, 1.0);
  node->get_parameter_or("Ki", Ki_, 0.0);
  node->get_parameter_or("Kd", Kd_, 0.0);
  node->get_parameter_or("upper_limit", upper_limit_, 1000.0);
  node->get_parameter_or("lower_limit", lower_limit_, -1000.0);
  node->get_parameter_or("windup_limit", windup_limit_, 1000.0);
  node->get_parameter_or("cutoff_frequency", cutoff_frequency_, -1.0);
  node->get_parameter_or("topic_from_controller", topic_from_controller_, "control_effort");
  node->get_parameter_or("topic_from_plant", topic_from_plant_, "state");
  node->get_parameter_or("setpoint_topic", setpoint_topic_, "setpoint");
  node->get_parameter_or("topic_pub_speed", topic_pub_speed_, "pub_speed");
  node->get_parameter_or("pid_enable_topic", pid_enable_topic_, "pid_enable");
  node->get_parameter_or("max_loop_frequency", max_loop_frequency_, 1.0);
  node->get_parameter_or("min_loop_frequency", min_loop_frequency_, 1000.0);
  node->get_parameter_or("pid_debug_topic", pid_debug_pub_name_, "pid_debug");
  node->get_parameter_or("setpoint_timeout", setpoint_timeout_, -1.0);

  if (setpoint_timeout_ == -1 || setpoint_timeout_ > 0)
  {
    RCLCPP_ASSERT(node->get_logger(), setpoint_timeout_ == -1 || setpoint_timeout_ > 0,
                  "setpoint_timeout set to %.2f but needs to -1 or >0", setpoint_timeout_);
  }

  // Two parameters to allow for error calculation with discontinous value
  node->get_parameter_or("angle_error", angle_error_, false);
  node->get_parameter_or("angle_wrap", angle_wrap_, 2.0 * 3.14159);

  // defaults diameter
  node->get_parameter_or("diameter", diameter_, -1.0);

  // Update params if specified as command-line options, & print settings
  printParameters();
  if (!validateParameters())
  {
    RCLCPP_ERROR(node->get_logger(), "Error: invalid parameter");
  }

  // instantiate publishers & subscribers
  control_effort_pub_ = this->create_publisher<std_msgs::msg::Float64>(topic_from_controller_, 1);
  pid_debug_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(pid_debug_pub_name_, 1);

  auto plant_sub = this->create_subscription<std_msgs::msg::Float64>(
      topic_from_plant_, 1, std::bind(&PidObject::plantStateCallback, this, std::placeholders::_1));
  auto setpoint_sub = this->create_subscription<std_msgs::msg::Float64>(
      setpoint_topic_, 1, std::bind(&PidObject::setpointCallback, this, std::placeholders::_1));
  auto pid_enabled_sub = this->create_subscription<std_msgs::msg::Bool>(
      pid_enable_topic_, 1, std::bind(&PidObject::pidEnableCallback, this, std::placeholders::_1));

  // speed publish m/s
  auto pub_speed = this->create_publisher<std_msgs::msg::Float64>(topic_pub_speed_, 10);

  if (!plant_sub_ || !setpoint_sub_ || !pid_enabled_sub_)
  {
    RCLCPP_ERROR(node->get_logger(), "Initialization of a subscriber failed. Exiting.");
    rclcpp::shutdown();
    std::exit(EXIT_FAILURE);
  }

  // dynamic reconfiguration
  auto config_server = this->create_service<pid_msgs::srv::SetPidConfig>(
      "set_pid_config", std::bind(&PidObject::setPidConfigCallback, this, std::placeholders::_1, std::placeholders::_2));

  // Wait for first messages
  while (!rclcpp::topic::wait_for_message<std_msgs::msg::Float64>(setpoint_topic_, std::chrono::seconds(10)))
  {
    RCLCPP_WARN(this->get_logger(), "Waiting for first setpoint message.");
  }

  while (!rclcpp::topic::wait_for_message<std_msgs::msg::Float64>(topic_from_plant_, std::chrono::seconds(10)))
  {
    RCLCPP_WARN(this->get_logger(), "Waiting for first state message from the plant.");
  }
  // Respond to inputs until shut down
  while (rclcpp::ok())
  {
    doCalcs();
    rclcpp::spin_some(shared_from_this());

    // publish speed m/s
    pub_speed->publish(speed_pub_);

    // Add a small sleep to avoid 100% CPU usage
    rclcpp::sleep_for(std::chrono::milliseconds(1));
  }
}
void PidObject::setpointCallback(const std_msgs::msg::Float64::SharedPtr setpoint_msg)
{
  setpoint_ = setpoint_msg->data;
  last_setpoint_msg_time_ = rclcpp::Clock().now();
  new_state_or_setpt_ = true;
}

void PidObject::plantStateCallback(const std_msgs::msg::Float64::SharedPtr state_msg)
{
  plant_state_ = state_msg->data / 2 * 0.17;
  speed_pub_.data = plant_state_;
  new_state_or_setpt_ = true;
}

void PidObject::pidEnableCallback(const std_msgs::msg::Bool::SharedPtr pid_enable_msg)
{
  pid_enabled_ = pid_enable_msg->data;
}

void PidObject::getParams(double in, double &value, double &scale)
{
  int digits = 0;
  value = in;
  while (rclcpp::ok() && ((std::fabs(value) > 1.0 || std::fabs(value) < 0.1) && (digits < 2 && digits > -1)))
  {
    if (std::fabs(value) > 1.0)
    {
      value /= 10.0;
      digits++;
    }
    else
    {
      value *= 10.0;
      digits--;
    }
  }
  if (value > 1.0)
    value = 1.0;
  if (value < -1.0)
    value = -1.0;

  scale = std::pow(10.0, digits);
}

bool PidObject::validateParameters()
{
  if (lower_limit_ > upper_limit_)
  {
    RCLCPP_ERROR("The lower saturation limit cannot be greater than the upper saturation limit.");
    return (false);
  }

  return true;
}

void PidObject::printParameters()
{
  RCLCPP_INFO(
      "PID PARAMETERS\n"
      "-----------------------------------------\n"
      "Kp: %f,  Ki: %f,  Kd: %f\n"
      "LPF cutoff frequency: %s\n"
      "pid node name: %s\n"
      "Name of topic from controller: %s\n"
      "Name of topic from the plant: %s\n"
      "Name of setpoint topic: %s\n"
      "Integral-windup limit: %f\n"
      "Saturation limits: %f/%f\n"
      "-----------------------------------------",
      Kp_, Ki_, Kd_,
      cutoff_frequency_ == -1 ? "1/4 of sampling rate" : std::to_string(cutoff_frequency_).c_str(),
      rclcpp::this_node::get_name(),
      topic_from_controller_.c_str(),
      topic_from_plant_.c_str(),
      setpoint_topic_.c_str(),
      windup_limit_,
      upper_limit_, lower_limit_);
}

void PidObject::reconfigureCallback(pid_msgs::msg::PidConfig &config, uint32_t level)
{
  if (first_reconfig_)
  {
    getParams(Kp_, config.Kp, config.Kp_scale);
    getParams(Ki_, config.Ki, config.Ki_scale);
    getParams(Kd_, config.Kd, config.Kd_scale);
    first_reconfig_ = false;
    return; // Ignore the first call to reconfigure which happens at startup
  }

  Kp_ = config.Kp * config.Kp_scale;
  Ki_ = config.Ki * config.Ki_scale;
  Kd_ = config.Kd * config.Kd_scale;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pid reconfigure request: Kp: %f, Ki: %f, Kd: %f", Kp_, Ki_, Kd_);
}
void PidObject::doCalcs()
{
  // Do fresh calcs if knowledge of the system has changed.
  if (new_state_or_setpt_)
  {
    if (!((Kp_ <= 0. && Ki_ <= 0. && Kd_ <= 0.) ||
          (Kp_ >= 0. && Ki_ >= 0. && Kd_ >= 0.))) // All 3 gains should have the same sign
      RCLCPP_WARN(ros::get_logger(), "All three gains (Kp, Ki, Kd) should have the same sign for stability.");

    error_[2] = error_[1];
    error_[1] = error_[0];
    error_[0] = setpoint_ - plant_state_; // Current error goes to slot 0

    // If the angle_error param is true, then address discontinuity in error
    // calc.
    // For example, this maintains an angular error between -180:180.
    if (angle_error_)
    {
      while (error_[0] < -1.0 * angle_wrap_ / 2.0)
      {
        error_[0] += angle_wrap_;

        // The proportional error will flip sign, but the integral error
        // won't and the filtered derivative will be poorly defined. So,
        // reset them.
        error_deriv_[2] = 0.;
        error_deriv_[1] = 0.;
        error_deriv_[0] = 0.;
        error_integral_ = 0.;
      }
      while (error_[0] > angle_wrap_ / 2.0)
      {
        error_[0] -= angle_wrap_;

        // The proportional error will flip sign, but the integral error
        // won't and the filtered derivative will be poorly defined. So,
        // reset them.
        error_deriv_[2] = 0.;
        error_deriv_[1] = 0.;
        error_deriv_[0] = 0.;
        error_integral_ = 0.;
      }
    }

    // calculate delta_t
    if (!prev_time_.isZero()) // Not first time through the program
    {
      delta_t_ = rclcpp::Clock().now() - prev_time_;
      prev_time_ = rclcpp::Clock().now();
      if (0 == delta_t_.seconds())
      {
        RCLCPP_ERROR(node->get_logger(), "delta_t is 0, skipping this loop. Possible overloaded CPU at time: %f",
                     rclcpp::Clock().now().seconds());
        return;
      }
    }
    else
    {
      RCLCPP_INFO(node->get_logger(), "prev_time is 0, doing nothing");
      prev_time_ = rclcpp::Clock().now();
      return;
    }

    // integrate the error
    error_integral_ += error_[0] * delta_t_.seconds();

    // Apply windup limit to limit the size of the integral term
    if (error_integral_ > fabs(windup_limit_))
      error_integral_ = fabs(windup_limit_);

    if (error_integral_ < -fabs(windup_limit_))
      error_integral_ = -fabs(windup_limit_);

    // My filter reference was Julius O. Smith III, Intro. to Digital Filters
    // With Audio Applications.
    // See https://ccrma.stanford.edu/~jos/filters/Example_Second_Order_Butterworth_Lowpass.html
    if (cutoff_frequency_ != -1)
    {
      // Check if tan(_) is really small, could cause c = NaN
      tan_filt_ = tan((cutoff_frequency_ * 6.2832) * delta_t_.seconds() / 2);

      // Avoid tan(0) ==> NaN
      if ((tan_filt_ <= 0.) && (tan_filt_ > -0.01))
        tan_filt_ = -0.01;
      if ((tan_filt_ >= 0.) && (tan_filt_ < 0.01))
        tan_filt_ = 0.01;

      c_ = 1 / tan_filt_;
    }

    filtered_error_[2] = filtered_error_[1];
    filtered_error_[1] = filtered_error_[0];
    filtered_error_[0] =
        (1 / (1 + c_ * c_ + 1.414 * c_)) * (error_[2] + 2 * error_[1] + error_[0] -
                                            (c_ * c_ - 1.414 * c_ + 1) * filtered_error_[2] -
                                            (-2 * c_ * c_ + 2) * filtered_error_[1]);

    // Take derivative of error
    // First the raw, unfiltered data:
    error_deriv_[2] = error_deriv_[1];
    error_deriv_[1] = error_deriv_[0];
    error_deriv_[0] = (error_[0] - error_[1]) / delta_t_.seconds();

    filtered_error_deriv_[2] = filtered_error_deriv_[1];
    filtered_error_deriv_[1] = filtered_error_deriv_[0];

    filtered_error_deriv_[0] =
        (1 / (1 + c_ * c_ + 1.414 * c_)) *
        (error_deriv_[2] + 2 * error_deriv_[1] + error_deriv_[0] -
         (c_ * c_ - 1.414 * c_ + 1) * filtered_error_deriv_[2] - (-2 * c_ * c_ + 2) * filtered_error_deriv_[1]);

    // calculate the control effort
    proportional_ = Kp_ * filtered_error_[0];
    integral_ = Ki_ * error_integral_;
    derivative_ = Kd_ * filtered_error_deriv_[0];
    control_effort_ = proportional_ + integral_ + derivative_;

    // Apply saturation limits
    if (control_effort_ > upper_limit_)
      control_effort_ = upper_limit_;
    else if (control_effort_ < lower_limit_)
      control_effort_ = lower_limit_;

    // Publish the stabilizing control effort if the controller is enabled
    if (pid_enabled_ && (setpoint_timeout_ == -1 ||
                         (rclcpp::Time::now() - last_setpoint_msg_time_).seconds() <= setpoint_timeout_))
    {
      control_msg_.data = control_effort_;
      control_effort_pub_->publish(control_msg_);
      // Publish topic with
      std::vector<double> pid_debug_vect{plant_state_, control_effort_, proportional_, integral_, derivative_};
      auto pidDebugMsg = std::make_unique<std_msgs::msg::Float64MultiArray>();
      pidDebugMsg->data = pid_debug_vect;
      pid_debug_pub_->publish(std::move(pidDebugMsg));
    }
    else if (setpoint_timeout_ > 0 &&
             (rclcpp::Time::now() - last_setpoint_msg_time_).seconds() > setpoint_timeout_)
    {
      RCLCPP_WARN_ONCE(node->get_logger(), "Setpoint message timed out, will stop publishing control_effort_messages");


      error_integral_ = 0.0;
    }
    else
      error_integral_ = 0.0;
  }

  new_state_or_setpt_ = false;
}
