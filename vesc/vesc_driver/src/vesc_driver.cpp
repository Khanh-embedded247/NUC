/*********************************************************************
 * mục đích của khi chạy file
1.Khởi tạo và Kết nối

2.Chương trình mở một kết nối với mô-đun VESC thông qua cổng nối tiếp (port)
 Nếu không thể kết nối, chương trình sẽ xuất lỗi và thoát.
 Xác định Số phiên bản Firmware:

3.Trong quá trình khởi tạo, chương trình yêu cầu số phiên bản firmware từ VESC và chờ đợi để nhận phản hồi.
 Số phiên bản này có thể quan trọng để đảm bảo tính tương thích với chương trình điều khiển.
 Chế Độ Hoạt Động:

4Chương trình chuyển sang chế độ hoạt động sau khi xác định số phiên bản firmware. 
Trong chế độ này,  liên tục thăm dò trạng thái của VESC và cập nhật thông tin về trạng thái của nó.
Xử lý Lệnh và Đăng Ký Publisher:

5.Chương trình đăng ký các chủ đề ROS để nhận lệnh từ người điều khiển bên ngoài
 thông qua các chủ đề như commands/motor/duty_cycle, commands/motor/current, commands/motor/brake, commands/motor/speed, commands/motor/position, và commands/servo/position.
Xử lý Gói Tin VESC:

6.Chương trình đăng ký hàm callback để xử lý các gói tin VESC, bao gồm trạng thái và phiên bản firmware.
Giới Hạn Lệnh:

7.Chương trình kiểm soát và giới hạn các lệnh được nhận từ người điều khiển dựa trên các thông số giới hạn như duty_cycle, current, brake, speed, position, và servo.
Xuất Trạng Thái và Lỗi:

8.Nếu có lỗi trong quá trình thực hiện, chương trình xuất lỗi và có thể chấm dứt quá trình.
cung cấp một cầu nối giữa ROS và mô-đun điều khiển VESC, cho phép người điều khiển và giám sát trạng thái của động cơ điện thông qua ROS.
 ********************************************************************/

// TODO: Migrate to ROS2
#include "/home/robotic/vesc/vesc_driver/include/vesc_driver/vesc_driver.h"
#include "std_msgs/msg/float64_multi_array.hpp"
namespace vesc_driver
{//dựa theo cấu trúc các lệnh VESC 

VescDriver::VescDriver(rclcpp::Node::SharedPtr nh, rclcpp::Node::SharedPtr private_nh)
  : vesc_(std::string(), std::bind(&VescDriver::vescPacketCallback, this, std::placeholders::_1),
          std::bind(&VescDriver::vescErrorCallback, this, std::placeholders::_1))
  , duty_cycle_limit_(private_nh, "duty_cycle", -1.0, 1.0)
  , current_limit_(private_nh, "current")
  , brake_limit_(private_nh, "brake")
  , speed_limit_(private_nh, "speed")
  , position_limit_(private_nh, "position")
  , servo_limit_(private_nh, "servo", 0.0, 1.0)
  , driver_mode_(MODE_INITIALIZING)
  , fw_version_major_(-1)
  , fw_version_minor_(-1)
{
  //////////////////////////////////////////////////////////////////////////////////
  // Lấy địa chỉ cổng vesc
  std::string port;
  if (!private_nh.getParam("port", port))
  {
    RCLCPP_FATAL(private_nh->get_logger(), "VESC communication port parameter required.");
    rclcpp::shutdown();
    return;
  }

  // kết nối tơi cổng nối tiếp nếu tìm dc thông tin cổng
  try
  {
    vesc_.connect(port);
  }
  catch (SerialException e)
  {
    RCLCPP_FATAL(private_nh->get_logger(), "Failed to connect to the VESC, %s.", e.what());
    rclcpp::shutdown();
    return;
  }
////////////////////////////////////////////////////////////////////////////////////
  // lấy số cặp cực động cơ
  private_nh.param("num_motor_pole_pairs", num_motor_pole_pairs_, 1);
  RCLCPP_INFO(private_nh->get_logger(), "The number of motor pole pairs is set to %d", num_motor_pole_pairs_);
///////////////////////////////////////////////////////////////////////////////////

  // tạo publisher trạng thái vesc
  state_pub_ = nh.advertise<vesc_msgs::VescStateStamped>("sensors/core", 10);

  // since vesc state does not include the servo position, publish the commanded
  // vị trí servo như 1 sensor
  servo_sensor_pub_ = nh.advertise<std_msgs::Float64>("sensors/servo_position_command", 10);

  // đăng ký các chủ đề về lệnh động cơ và servo
  duty_cycle_sub_ = nh->create_subscription<std_msgs::msg::Float64>("commands/motor/duty_cycle", 10,
                                                                    std::bind(&VescDriver::dutyCycleCallback, this, std::placeholders::_1));
  current_sub_ = nh->create_subscription<std_msgs::msg::Float64>("commands/motor/current", 10,
                                                                std::bind(&VescDriver::currentCallback, this, std::placeholders::_1));
  brake_sub_ = nh->create_subscription<std_msgs::msg::Float64>("commands/motor/brake", 10,
                                                              std::bind(&VescDriver::brakeCallback, this, std::placeholders::_1));
  speed_sub_ = nh->create_subscription<std_msgs::msg::Float64>("commands/motor/speed", 10,
                                                              std::bind(&VescDriver::speedCallback, this, std::placeholders::_1));
  position_sub_ = nh->create_subscription<std_msgs::msg::Float64>("commands/motor/position", 10,
                                                                  std::bind(&VescDriver::positionCallback, this, std::placeholders::_1));
  servo_sub_ = nh->create_subscription<std_msgs::msg::Float64>("commands/servo/position", 10,
                                                               std::bind(&VescDriver::servoCallback, this, std::placeholders::_1));

  //tạo bộ đếm thời gian 50Hz, được sử dụng cho máy trạng thái và đo từ xa VESC bỏ phiếu
  timer_ = nh->create_wall_timer(std::chrono::milliseconds(20), std::bind(&VescDriver::timerCallback, this));
}
/**********************************************************************************************************/
/* DANH SÁCH VIỆC CẦN LÀM hoặc CẦN NGHĨ
  - chúng ta nên làm gì khi khởi động? gửi lệnh phanh hoặc lệnh không?
  - phải làm gì nếu giao diện vesc báo lỗi?
  - kiểm tra số phiên bản có biết tương thích không?
  - chúng ta có nên đợi cho đến khi nhận được dữ liệu đo từ xa trước khi gửi lệnh không?
  - chúng ta có nên theo dõi lệnh động cơ cuối cùng không
  - phải làm gì nếu gần đây không nhận được lệnh động cơ?
  - phải làm gì nếu gần đây không nhận được lệnh servo?
  - trạng thái tắt an toàn của động cơ (0 hiện tại?)
  - phải làm gì nếu tham số lệnh nằm ngoài phạm vi, bỏ qua?
  - cố gắng dự đoán giới hạn vesc (từ cấu hình vesc) và lệnh phát hiện lỗi giới hạn
**********************************************************************************************************/

void VescDriver::timerCallback(const ros::TimerEvent& event)
{
  // Giao diện VESC không được ngắt kết nối đột ngột nhưng vẫn phải kiểm tra nó
  if (!vesc_.isConnected())
  { 
    RCLCPP_FATAL(private_nh->get_logger(), "Unexpectedly disconnected from serial port.");
    timer_.cancel();
    rclcpp::shutdown();
    return;
  }

  /*
   * Trình điều khiển trạng thái máy, các chế độ:
   * KHỞI TẠO - yêu cầu và chờ phiên bản vesc
   * VẬN HÀNH - nhận lệnh từ các topic 
   */


  //Nếu chế đọ lái khởi tạo 
  if (driver_mode_ == MODE_INITIALIZING)
  {
    // yêu cầu số phiên bản, gói trả về sẽ cập nhật số phiên bản của firmware
    vesc_.requestFWVersion();
    if (fw_version_major_ >= 0 && fw_version_minor_ >= 0)
    {
      RCLCPP_INFO(private_nh->get_logger(), "Connected to VESC with firmware version %d.%d", fw_version_major_, fw_version_minor_);
      driver_mode_ = MODE_OPERATING;
    }
  }
  //nẾU CHẾ ĐỘ VÂN HÀNH
  else if (driver_mode_ == MODE_OPERATING)
  {
    //thăm dò trạng thái vesc (đo từ xa)
    vesc_.requestState();
  }
  else
  {
    // Chế độ không xác định (hàm assert :trả về dạng thông váo khi không tìm thấy đối số của macro dưới dạng so sánh)
    assert(false && "unknown driver mode");
  }
}

void VescDriver::vescPacketCallback(const std::shared_ptr<VescPacket const>& packet)
{
  if (packet->getName() == "Values")
  {//truyền gói dữ liệu sub dc đến gói VescPacketValues
    std::shared_ptr<VescPacketValues const> values = std::dynamic_pointer_cast<VescPacketValues const>(packet);
   //tạo con trỏ duy nhất đến tin nhắn VescStateStamped(auto có nghĩa c+11 > tạo các ms )
    auto state_msg = std::make_unique<vesc_msgs::msg::VescStateStamped>();
    //viết thông báo VescStateStamped từ gói VescPacketValues
    state_msg->header.stamp = ros_node_->get_clock()->now();//đặt dáu thời gian bằng real
    state_msg->state.voltage_input = values->getInputVoltage();//giá trị điện nang tiêu thụ 
    state_msg->state.temperature_pcb = values->getMosTemp();//nhiệt độ 
    state_msg->state.current_motor = values->getMotorCurrent();//đặt dòng điện động cơ cần
    state_msg->state.current_input = values->getInputCurrent();//đặt điện áp  vào
    //tính tần số cặp cực quay được bao nhiêu vòng/phút(tổng số vòng đô dc ở encorder/số cặp cực /chu vi hình tròn)
    state_msg->state.speed = values->getVelocityERPM() / static_cast<double>(num_motor_pole_pairs_) / 60.0 * 2.0 * M_PI;
    state_msg->state.duty_cycle = values->getDuty();//độ rộng xung
    state_msg->state.charge_drawn = values->getConsumedCharge();
    state_msg->state.charge_regen = values->getInputCharge();
    state_msg->state.energy_drawn = values->getConsumedPower();//điện năng tiêu thụ
    state_msg->state.energy_regen = values->getInputPower();//nguồn didenj đầu vào
    state_msg->state.displacement = values->getPosition();//vị trí
    state_msg->state.distance_traveled = values->getDisplacement();//sự dịch chuyển
    state_msg->state.fault_code = values->getFaultCode();//lấy mã lỗi 
  //publish tin nhắn
    state_pub_->publish(std::move(state_msg));
  }
  else if (packet->getName() == "FWVersion")
  {
    std::shared_ptr<VescPacketFWVersion const> fw_version =
        std::dynamic_pointer_cast<VescPacketFWVersion const>(packet);
   //lấy thông tin về phiên bản firmware từ VescPacketFWVersion
    fw_version_major_ = fw_version->fwMajor();
    fw_version_minor_ = fw_version->fwMinor();
  }
}

void VescDriver::vescErrorCallback(const std::string& error)
{
  ROS_ERROR("%s", error.c_str());
}

/**
 * @param duty_cycle chu kỳ nhiệm vụ VESC được yêu cầu. Phạm vi hợp lệ cho trình điều khiển này là -1 đến +1. Tuy nhiên,
 * lưu ý rằng VESC có thể áp đặt giới hạn hạn chế hơn đối với phạm vi tùy thuộc vào
 * trên cấu hình của nó, ví dụ: giá trị tuyệt đối nằm trong khoảng từ 0,05 đến 0,95.
 */
void VescDriver::dutyCycleCallback(const std_msgs::Float64::ConstPtr& duty_cycle)
{
  if (driver_mode_ = MODE_OPERATING)
  {
    vesc_.setDutyCycle(duty_cycle_limit_.clip(duty_cycle->data));
  }
}

/**
 * @param currentLệnh VESC hiện tại trong Amps. Bất kỳ giá trị nào đều được trình điều khiển này chấp nhận. Tuy nhiên,
 * lưu ý rằng VESC có thể áp đặt giới hạn hạn chế hơn đối với phạm vi tùy thuộc vào
 * cấu hình của nó.
 */
 
void VescDriver::currentCallback(const std_msgs::Float64::ConstPtr& current)
{
  if (driver_mode_ = MODE_OPERATING)
  {
    vesc_.setCurrent(current_limit_.clip(current->data));
  }
}

/**
 * @param brake Lệnh hãm VESC hiện tại trong Amps. Bất kỳ giá trị nào đều được trình điều khiển này chấp nhận.
 * Tuy nhiên, lưu ý rằng VESC có thể áp đặt giới hạn hạn chế hơn đối với phạm vi
 * tùy thuộc vào cấu hình của nó.
 */
void VescDriver::brakeCallback(const std_msgs::Float64::ConstPtr& brake)
{
  if (driver_mode_ = MODE_OPERATING)
  {
    vesc_.setBrake(brake_limit_.clip(brake->data));
  }
}

/**
 * @param speed Commanded VESC speed in rad/s. Although any value is accepted by this driver, the VESC may impose a
 *              more restrictive bounds on the range depending on its configuration.
 */
void VescDriver::speedCallback(const std_msgs::Float64::ConstPtr& speed)
{
  if (driver_mode_ = MODE_OPERATING)
  {
    const double speed_erpm = speed->data / 2.0 / M_PI * 60.0 * static_cast<double>(num_motor_pole_pairs_);
    vesc_.setSpeed(speed_limit_.clip(speed_erpm));
  }
}

/**
 * @param position Commanded VESC motor position in radians. Any value is accepted by this driver.
 *                 Note that the VESC must be in encoder mode for this command to have an effect.
 */
void VescDriver::positionCallback(const std_msgs::Float64::ConstPtr& position)
{
  if (driver_mode_ = MODE_OPERATING)
  {
    // ROS uses radians but VESC seems to use degrees. Convert to degrees.
    double position_deg = position_limit_.clip(position->data) * 180.0 / M_PI;
    vesc_.setPosition(position_deg);
  }
}

/**
 * @param servo Commanded VESC servo output position. Valid range is 0 to 1.
 */
void VescDriver::servoCallback(const std_msgs::Float64::ConstPtr& servo)
{
  if (driver_mode_ = MODE_OPERATING)
  {
    double servo_clipped(servo_limit_.clip(servo->data));
    vesc_.setServo(servo_clipped);
    // publish clipped servo value as a "sensor"
    std_msgs::Float64::Ptr servo_sensor_msg(new std_msgs::Float64);
    servo_sensor_msg->data = servo_clipped;
    servo_sensor_pub_.publish(servo_sensor_msg);
  }
}
/*******************************************************************************************************************/
VescDriver::CommandLimit::CommandLimit(const ros::NodeHandle& nh, const std::string& str,
                                       const boost::optional<double>& min_lower,
                                       const boost::optional<double>& max_upper)
  : name(str)
{
  // kiểm tra xem giá trị tối thiểu của người dùng có nằm ngoài phạm vi min_low đến max_upper không
  double param_min;
  if (nh.getParam(name + "_min", param_min))
  {
    if (min_lower && param_min < *min_lower)
    {
      lower = *min_lower;
      RCLCPP_WARN(node->get_logger(), "Parameter %s_min (%f) is less than the feasible minimum (%f).",
                  name.c_str(), param_min, *min_lower);
    }
    else if (max_upper && param_min > *max_upper)
    {
      lower = *max_upper;
      RCLCPP_WARN(node->get_logger(), "Parameter %s_min (%f) is greater than the feasible maximum (%f).",
                  name.c_str(), param_min, *max_upper);
    }
    else
    {
      lower = param_min;
    }
  }
  else if (min_lower)
  {
    lower = *min_lower;
  }

  // check if the uers' maximum value is outside of the range min_lower to max_upper
  double param_max;
  if (nh.getParam(name + "_max", param_max))
  {
    if (min_lower && param_max < *min_lower)
    {
      upper = *min_lower;
      RCLCPP_WARN(node->get_logger(), "Parameter %s_max (%f) is less than the feasible minimum (%f).",
                  name.c_str(), param_max, *min_lower);
    }
    else if (max_upper && param_max > *max_upper)
    {
      upper = *max_upper;
      RCLCPP_WARN(node->get_logger(), "Parameter %s_max (%f) is greater than the feasible maximum (%f).",
                  name.c_str(), param_max, *max_upper);
    }
    else
    {
      upper = param_max;
    }
  }
  else if (max_upper)
  {
    upper = *max_upper;
  }

  // check for min > max
  if (upper && lower && *lower > *upper)
  {
    RCLCPP_WARN(node->get_logger(), "Parameter %s_max (%f) is less than parameter %s_min (%f).",
                name.c_str(), *upper, name.c_str(), *lower);
    double temp(*lower);
    lower = *upper;
    upper = temp;
  }

  RCLCPP_DEBUG(node->get_logger(), "  %s limit: %s%s%s", name.c_str(),
               lower ? std::to_string(*lower).c_str() : "(none)",
               lower && upper ? " " : "",
               upper ? std::to_string(*upper).c_str() : "(none)");
}

double VescDriver::CommandLimit::clip(double value)
{
  if (lower && value < lower)
  {
    RCLCPP_INFO_THROTTLE(10, "%s command value (%f) below minimum limit (%f), clipping.",
                         name.c_str(), value, *lower);
    return *lower;
  }
  if (upper && value > upper)
  {
   RCLCPP_INFO_THROTTLE(10, "%s command value (%f) above maximum limit (%f), clipping.",
                         name.c_str(), value, *upper);
    return *upper;
  }
  return value;
}

}  // namespace vesc_driver
