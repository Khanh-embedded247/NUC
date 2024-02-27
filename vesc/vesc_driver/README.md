#Thư viện lái xe VESC
`vesc_driver` là thư viện cơ bản để điều khiển động cơ bằng VESC.

## Đặc trưng
- giao tiếp với VESC.
- Hỗ trợ điều khiển vị trí (PID), vận tốc, dòng điện và chu kỳ nhiệm vụ.
  - Hiệu chuẩn và giới hạn khớp được thực hiện để kiểm soát vị trí.
  - Bạn có thể đưa ra hằng số mô-men xoắn và tỷ số truyền của động cơ giảm tốc

## Cấu trúc gói
#### Sơ đồ phụ thuộc
```
vesc_driver
 -> vesc_interface
    |-> vesc_packet_factory
    |-> vesc_packet
```

#### Sự miêu tả
- `vesc_packet` xác định các gói truyền thông và giao diện thấp nhất thông qua cổng nối tiếp USB.
- `vesc_packet_factory` tạo các gói được định nghĩa trong `vesc_packet`.
- `vesc_interface` cung cấp cho chúng ta các API cơ bản để điều khiển trình điều khiển motor VESC: Các hàm API gửi lệnh bao gồm nhiệm vụ, dòng điện tham chiếu, phanh và vận tốc tham chiếu.
- `vesc_driver` xuất bản tất cả các trạng thái bằng cách sử dụng `vesc_msgs` và gửi lệnh với các chức năng gọi lại. ``


## Cài đặt
Gói này phụ thuộc vào các gói ROS sau.

- nút nhỏ
- pluginlib
- roscpp
- std_msgs
- nối tiếp

Trước khi xây dựng cái này, bạn phải cài đặt chúng (ví dụ: bằng lệnh `apt`).

## Cách sử dụng
sẽ được chuẩn bị...

## Giấy phép
`vesc_driver` được cấp phép theo [giấy phép Apache 2.0](https://www.apache.org/licenses/LICENSE-2.0.html).