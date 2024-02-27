/***************************************************************************/
/**
 * \file autotune.cpp
 *
 * \brief Autotune a PID controller with the Ziegler Nichols method
 * \author Andy Zelenak
 * \date October 25, 2016
 *
 * \section license License (BSD-3)
 
 ******************************************************************************/

#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <string>

void setKiKdToZero(rclcpp::Node::SharedPtr node);
void setKp(rclcpp::Node::SharedPtr node, double Kp);
void setpointCallback(const std_msgs::msg::Float64::SharedPtr setpoint_msg);
void stateCallback(const std_msgs::msg::Float64::SharedPtr state_msg);
void setFinalParams(rclcpp::Node::SharedPtr node);

namespace autotune
{
double Ku = 0.;
double Tu = 0.;
double setpoint = 0.;
double state = 0.;
std::string ns = "/left_wheel_pid/";
int oscillation_count = 0;
int num_loops = 100;  // // Sẽ tìm dao động cho num_loops*loopRate
int initial_error = 0;
double Kp_ZN = 0.;
double Ki_ZN = 0.;
double Kd_ZN = 0.;
bool found_Ku = false;
std::vector<double> oscillation_times(3);  // Dùng để tính Tu, chu kỳ dao động
}  // namespace autotune

class AutotuneNode : public rclcpp::Node
{
public:
  AutotuneNode() : Node("autotune_node")
  {
    setpoint_sub_ = create_subscription<std_msgs::msg::Float64>("setpoint", 1, setpointCallback);
    state_sub_ = create_subscription<std_msgs::msg::Float64>("state", 1, stateCallback);

    // Set Ki and Kd to zero for the ZN method
    setKiKdToZero(shared_from_this());

    // Xác định tốc độ thay đổi của giá trị Kp và các giá trị tối đa/tối thiểu cần thử
    double Kp_max = 10.;
    double Kp_min = 0.5;
    double Kp_step = 1.0;

    for (double Kp = Kp_min; Kp <= Kp_max; Kp += Kp_step)
    {
      //////////////////////
      // Get the error sign.
      //////////////////////
      // Need to wait for new setpoint/state msgs
      rclcpp::spin_some(shared_from_this());
      rclcpp::spin_some(shared_from_this());

      // Try a new Kp.
      setKp(shared_from_this(), Kp);
      RCLCPP_INFO_STREAM(get_logger(), "Trying Kp = " << Kp);  // Dòng trống trên thiết bị đầu cuối
      autotune::oscillation_count = 0;                           // Đặt lại để tìm lại dao động

      for (int i = 0; i < autotune::num_loops; i++)  // Thu thập dữ liệu cho loopRate*num_loops giây
      {
        rclcpp::spin_some(shared_from_this());
        loop_rate_.sleep();
        if (i == 0)  // Nhận dấu hiệu của lỗi ban đầu
        {
          autotune::initial_error = (autotune::setpoint - autotune::state);
        }

        // Hệ thống có dao động quanh điểm đặt không? Nếu vậy thì Kp~Ku.
        // Dao động => dấu hiệu lỗi thay đổi
        // Dao động đầu tiên được gây ra bởi sự thay đổi điểm đặt. Bỏ mặc nó.
        // Tìm 2 dao động.
        // Nhận thông báo trạng thái mới
        rclcpp::spin_some(shared_from_this());
        double new_error = (autotune::setpoint - autotune::state);  // Sign of the error
        // RCLCPP_INFO_STREAM(get_logger(), "New error: "<< new_error);
        if (std::signbit(autotune::initial_error) != std::signbit(new_error))
        {
          autotune::oscillation_times.at(autotune::oscillation_count) =
              loop_rate_.expected_cycle_time().seconds() * i;  // Record the time to calculate a period, Tu
          autotune::oscillation_count++;
          RCLCPP_INFO_STREAM(get_logger(), "Oscillation occurred. Oscillation count:  " << autotune::oscillation_count);
          autotune::initial_error = new_error;  // Đặt lại để tìm dao động khác
          // Nếu hệ thống chắc chắn dao động quanh điểm đặt
          if (autotune::oscillation_count > 2)
          {
            // Hãy tính chu kỳ dao động (Tu)
            autotune::Tu = autotune::oscillation_times.at(2) - autotune::oscillation_times.at(0);
            RCLCPP_INFO_STREAM(get_logger(), "Tu (oscillation period): " << autotune::Tu);
            // RCLCPP_INFO_STREAM( "2*thời gian lấy mẫu: " <<
            // 2.*loop_rate_.expected_cycle_time().seconds() );

            // Chúng tôi đang tìm kiếm nhiều thứ hơn là chỉ là khoảng thời gian ngắn nhất trên
            // điểm đặt và quay lại.
            // Muốn thấy dao động đáng kể
            if (autotune::Tu > 3. * loop_rate_.expected_cycle_time().seconds())
            {
              autotune::Ku = Kp;

              //Bây giờ tính các tham số khác bằng phương pháp ZN
              autotune::Kp_ZN = 0.6 * autotune::Ku;
              autotune::Ki_ZN = 1.2 * autotune::Ku / autotune::Tu;
              autotune::Kd_ZN = 3. * autotune::Ku * autotune::Tu / 40.;

              autotune::found_Ku = true;
              goto DONE;
            }
            else
              break;  // Try the next Kp
          }
        }
      }
    }
  DONE:

    if (autotune::found_Ku == true)
    {
      setFinalParams(shared_from_this());
    }
    else
      RCLCPP_INFO_STREAM(get_logger(), "Did not see any oscillations for this range of Kp. Adjust "
                                       "Kp_max and Kp_min to broaden the search.");
  }

private:
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr setpoint_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr state_sub_;
  rclcpp::Rate loop_rate_{50};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutotuneNode>());
  rclcpp::shutdown();
  return 0;
}

// Set Ki and Kd to zero
void setKiKdToZero(rclcpp::Node::SharedPtr node)
{
  // Set Ki and Kd to zero by setting parameters
  node->set_parameter(rclcpp::Parameter("Ki", 0.0));
  node->set_parameter(rclcpp::Parameter("Kd", 0.0));
}

// Set Kp with dynamic_reconfigure
void setKp(rclcpp::Node::SharedPtr node, double Kp)
{
  // Set Kp parameter
  node->set_parameter(rclcpp::Parameter("Kp", Kp));
}

void setpointCallback(const std_msgs::msg::Float64::SharedPtr setpoint_msg)
{
  autotune::setpoint = setpoint_msg->data;
}

void stateCallback(const std_msgs::msg::Float64::SharedPtr state_msg)
{
  autotune::state = state_msg->data;
}

// Print out and set the final parameters as calculated by the autotuner
void setFinalParams(rclcpp::Node::SharedPtr node)
{
  RCLCPP_INFO_STREAM(node->get_logger(), " ");
  RCLCPP_INFO_STREAM(node->get_logger(), "The suggested parameters are: ");
  RCLCPP_INFO_STREAM(node->get_logger(), "Kp  " << autotune::Kp_ZN);
  RCLCPP_INFO_STREAM(node->get_logger(), "Ki  " << autotune::Ki_ZN);
  RCLCPP_INFO_STREAM(node->get_logger(), "Kd  " << autotune::Kd_ZN);

  // Set the ZN parameters
  node->set_parameter(rclcpp::Parameter("Kp", autotune::Kp_ZN));
  node->set_parameter(rclcpp::Parameter("Ki", autotune::Ki_ZN));
  node->set_parameter(rclcpp::Parameter("Kd", autotune::Kd_ZN));
}
