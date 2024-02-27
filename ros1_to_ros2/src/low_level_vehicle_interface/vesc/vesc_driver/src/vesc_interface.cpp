/*********************************************************************
 `vesc_interface` cung cấp  các API cơ bản để điều khiển trình điều khiển motor VESC:
  Các hàm API gửi lệnh bao gồm nhiệm vụ, dòng điện tham chiếu, phanh và vận tốc tham chiếu.


 ********************************************************************/

#include "vesc_driver/vesc_interface.hpp"
#include <serial_driver/serial_driver.hpp>
#include <rclcpp/rclcpp.hpp>

namespace vesc_driver
{//quản lý một luồng chuyên dụng ( rx_thread_) để liên tục nhận dữ liệu từ VESC
//Thread đọc dữ liệu từ cổng nối tiếp, xử lý nó và gọi trình xử lý gói để tìm các gói hợp lệ.
// Lớp này cũng bao gồm các cài đặt cấu hình, xử lý lỗi và cờ ( data_updated_) để cho biết liệu dữ liệu mới đã được nhận hay chưa.
class VescInterface::Impl
{
public:
//Một IoContext đối tượng được quản lý bởi một con trỏ duy nhất k thể sao chép sang 1 cái khác
// Nó được khởi tạo với một IoContext phiên bản mới có kích thước được chỉ định (2).
  Impl() : owned_ctx(new IoContext(2)), serial_driver_(new drivers::serial_driver::SerialDriver(*owned_ctx))
  {
    data_updated_ = false;
  }

  void* rxThread(void);//Mã định danh pthread đại diện cho luồng xử lý việc nhận dữ liệu từ VESC.

  static void* rxThreadHelper(void* context)
  {
    return ((VescInterface::Impl*)context)->rxThread();
  }

  pthread_t rx_thread_;
  bool rx_thread_run_;//Cờ boolean cho biết liệu luồng tiếp nhận có tiếp tục chạy hay không.
  // PacketHandlerFunction packet_handler_;/Một con trỏ hàm hoặc hàm gọi lại để xử lý các gói VESC
  ErrorHandlerFunction error_handler_;//Một con trỏ hàm hoặc gọi lại để xử lý lỗi trong quá trình xử lý gói.
  std::unique_ptr<IoContext> owned_ctx{};
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;//Một đối tượng cấu hình cho cài đặt cổng nối tiếp
  VescFrame::CRC send_crc_;//Đại diện cho CRC (Kiểm tra dự phòng theo chu kỳ) để gửi dữ liệu.
  bool data_updated_;
  PacketHandlerFunction packet_handler_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
   void setPacketHandler(const PacketHandlerFunction& handler)
  {
    packet_handler_ = handler;
  }
};

void* VescInterface::Impl::rxThread(void)
{
  Buffer buffer;
  buffer.reserve(4096);
  auto temp_buffer = Buffer(4096);
/**************************************************************************************************************
Vòng while (rx_thread_run_)lặp liên tục đọc dữ liệu từ cổng nối tiếp và xử lý nó.
Nó kiểm tra số byte yêu cầu tối thiểu ( VescFrame::VESC_MIN_FRAME_SIZE) và xử lý lỗi nếu thao tác đọc không thành công
Bộ đệm được tìm kiếm các ký tự bắt đầu khung hợp lệ và cố gắng tạo các gói VESC.
Nếu một gói hợp lệ được tạo, nó sẽ cập nhật trạng thái, gọi trình xử lý gói và tiếp tục xử lý.
Nếu cần nhiều byte hơn để tạo thành một gói hoàn chỉnh, vòng lặp sẽ ngắt để chờ dữ liệu bổ sung.
Vòng lặp tiếp tục cho đến khi rx_thread_run_trở thành false, biểu thị yêu cầu

****************************************************************************************************************/
  while (rx_thread_run_)//đọc dl liên tục từ cồng nối tiếp 
  {
    int bytes_needed = VescFrame::VESC_MIN_FRAME_SIZE;//khởi tạo KÍCH THƯỚC KHUNG VESC TỐI THIỂU
    //cố gắng đọc ít nhất các byte yêu cầu từ cổng nối tiếp
    const auto bytes_read = serial_driver_->port()->receive(temp_buffer);
    //dữ trữ 1 vùng bộ đệm và chèn liên tục với dl đọc dược trnahs tốn bộ nhớ
    buffer.reserve(buffer.size() + bytes_read);
    //sau đó đọc được dữ liệu lưu vào bộ nhớ đệm 
    buffer.insert(buffer.end(), temp_buffer.begin(), temp_buffer.begin() + bytes_read);
    // RCLCPP_INFO(rclcpp::get_logger("VescDriver"), "Read packets: %d", bytes_read);
    //Nếu đọc k đủ byte hoặc bộ đệm k đủ trống thì báo lỗi 
    if (bytes_needed > 0 && 0 == bytes_read && !buffer.empty())
    {
      error_handler_("Possibly out-of-sync with VESC, read timout in the middle of a frame.");
    }
    if (!buffer.empty())
    {
      // tìm kiếm bộ đệm cho các gói hợp lệ
      Buffer::iterator iter(buffer.begin());
      Buffer::iterator iter_begin(buffer.begin());
      while (iter != buffer.end())
      {
        //ktra các byte đầu khung có hợp lệ không
        if (VescFrame::VESC_SOF_VAL_SMALL_FRAME == *iter || VescFrame::VESC_SOF_VAL_LARGE_FRAME == *iter)
        {
          
          std::string error;
          VescPacketConstPtr packet = VescPacketFactory::createPacket(iter, buffer.end(), &bytes_needed, &error);
          if (packet)
          {
            // nếu hợp update dl
            data_updated_ = true;
            //ktra xem có dl đầu khung k hoepj lệ nào k 
            if (std::distance(iter_begin, iter) > 0)
            {
              std::ostringstream ss;
              ss << "Out-of-sync with VESC, unknown data leading valid frame. Discarding "
                 << std::distance(iter_begin, iter) << " bytes.";
              error_handler_(ss.str());
            }
            // xử lý gói
            packet_handler_(packet);
            // update state
            iter = iter + packet->getFrame().size();
            iter_begin = iter;
            // continue to look for another frame in buffer
            continue;
          }
          else if (bytes_needed > 0)
          {
            // need more data, break out of while loop
            break;  // for (iter_sof...
          }
          else
          {
            // nếu dl k thuộc gói ,next dl mới và báo lỗi
            error_handler_(error);
          }
        }

        iter++;
      }

      //nếu iter ở cuối bộ đệm thì cần nhiều byte hơn
      if (iter == buffer.end())
        bytes_needed = VescFrame::VESC_MIN_FRAME_SIZE;

      // xóa bộ đệm "đã sử dụng"
      if (std::distance(iter_begin, iter) > 0)
      {
        std::ostringstream ss;
        ss << "Out-of-sync with VESC, discarding " << std::distance(iter_begin, iter) << " bytes.";
        error_handler_(ss.str());
      }
      buffer.erase(buffer.begin(), iter);
    }
  }
}

VescInterface::VescInterface(const std::string& port, const PacketHandlerFunction& packet_handler,
                             const ErrorHandlerFunction& error_handler)
  : impl_(new Impl())
{
  setPacketHandler(packet_handler);
  setErrorHandler(error_handler);
  // kết nối port nếu disconect
  if (!port.empty())
    connect(port);
}

VescInterface::~VescInterface()
{
  //Dừng đcơ
  setDutyCycle(0.0);

  disconnect();
}

void VescInterface::setPacketHandler(const PacketHandlerFunction& handler)
{
  // todo - definately need mutex
  impl_->packet_handler_ = handler;
}

void VescInterface::setErrorHandler(const ErrorHandlerFunction& handler)
{
  // todo - definately need mutex
  impl_->error_handler_ = handler;
}

void VescInterface::connect(const std::string& port)
{
  // todo - mutex?

  if (isConnected())
  {
    throw SerialException("Already connected to serial port.");
  }

  // connect to serial port
  try
  {
    const uint32_t baud_rate = 115200;
    const auto fc = drivers::serial_driver::FlowControl::NONE;
    const auto pt = drivers::serial_driver::Parity::NONE;
    const auto sb = drivers::serial_driver::StopBits::ONE;
    impl_->device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
    impl_->serial_driver_->init_port(port, *impl_->device_config_);
    //kiểm tra xem port có sd k nếu k thì dùng
    if (!impl_->serial_driver_->port()->is_open())
    {
      impl_->serial_driver_->port()->open();
    }
  }
  catch (const std::exception& e)
  {
    std::stringstream ss;
    ss << "Failed to open the serial port to the VESC. " << e.what();
    throw SerialException(ss.str().c_str());
  }

  //giám sát khởi tạo 1 luồng 
  impl_->rx_thread_run_ = true;
  int result = pthread_create(&impl_->rx_thread_, NULL, &VescInterface::Impl::rxThreadHelper, impl_.get());
  assert(0 == result);//kiểm tra xem tạo topic pthread_Create có thành công k 
}

void VescInterface::disconnect()
{
  // todo - mutex?

  if (isConnected())
  {
    // bring down read thread
    impl_->rx_thread_run_ = false;
    int result = pthread_join(impl_->rx_thread_, NULL);
    assert(0 == result);

    impl_->serial_driver_->port()->close();
  }
}

bool VescInterface::isConnected() const
{
  auto port = impl_->serial_driver_->port();

  if (port)
  {
    return port->is_open();
  }
  else
  {
    return false;
  }
}
//update data
bool VescInterface::isRxDataUpdated() const
{
  bool output = impl_->data_updated_;
  impl_->data_updated_ = false;
  return output;
}
//gửi gói VESC qua port
void VescInterface::send(const VescPacket& packet)
{
  std::size_t written = impl_->serial_driver_->port()->send(packet.getFrame());
  if (written != packet.getFrame().size())
  {
    std::stringstream ss;
    ss << "Wrote " << written << " bytes, expected " << packet.getFrame().size() << ".";
    throw SerialException(ss.str().c_str());
  }
}

void VescInterface::requestFWVersion()//hàm trả về phiên bản
{
  send(VescPacketRequestFWVersion());
}

void VescInterface::requestState()//hàm trả về trạng thái từ xa
{
  send(VescPacketRequestValues());
}

void VescInterface::setDutyCycle(double duty_cycle)//Đặt chu kỳ hoạt động cho điều khiển động cơ.
{
  send(VescPacketSetDuty(duty_cycle));
}

void VescInterface::setCurrent(double current)//dặt điện áp
{
  send(VescPacketSetCurrent(current));
}

void VescInterface::setBrake(double brake)//đặt phanh
{
  send(VescPacketSetCurrentBrake(brake));
}

void VescInterface::setSpeed(double speed)//đặt tốc độ chỉ định
{
  send(VescPacketSetVelocityERPM(speed));
}

void VescInterface::setPosition(double position)//đặt vị trí mục tiêu
{
  send(VescPacketSetPos(position));
}

void VescInterface::setServo(double servo)//đặt vị trí servo chỉ định 
{
  send(VescPacketSetServoPos(servo));
}

}  // namespace vesc_driver