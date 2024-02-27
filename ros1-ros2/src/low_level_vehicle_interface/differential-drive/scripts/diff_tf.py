#!/usr/bin/env python

"""
   diff_tf.py - follows the output of a wheel encoder and
   creates tf and odometry messages.
   some code borrowed from the arbotix diff_controller script
   A good reference: http://rossum.sourceforge.net/papers/DiffSteer/
"""
"đọc dữ liệu từ hai bánh xe của robot (trái và phải) thông qua các thông điệp từ các encoder (lwheel và rwheel)."
"Nút này sử dụng dữ liệu encoder để tính toán vị trí và tốc độ của robot, sau đó xuất ra thông điệp odometry và transform (tf)."
import rclpy
from geometry_msgs.msg import Quaternion, Twist
"""Quaternion:Hệ tứ phương (x,y,z,w) theo dõi,áp dụng phép quay.Độ lớn luôn bằng 1 ,nếu khác 1 sẽ cảnh báo .
Để trnahs cảnh báo q.normalize();"""
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64
from tf2_ros import TransformBroadcaster
from math import sin, cos, pi

class DiffTf:

    def __init__(self):
        rclpy.init()
        "tạo node difff_tf"
        self.node = rclpy.create_node("diff_tf")
        self.nodename = self.node.get_name()
        self.node.get_logger().info("-I- %s started" % self.nodename)

        # Parameters
        self.rate = self.node.declare_parameter('rate', 30.0).value#tần suất cập nhật
        self.left_ticks_meter = self.node.declare_parameter('left_ticks_meter', 50).value#Số xung encoder cho mỗi mét cho bánh trái 
        self.right_ticks_meter = self.node.declare_parameter('right_ticks_meter', 50).value#Số xung encoder cho mỗi mét cho  bánh phải.
        self.base_width = self.node.declare_parameter('base_width', 1).value#khoảng cách giữa hai bánh xe (đon nvij 1 với góc quay hình tròn)
        self.base_frame_id = self.node.declare_parameter('base_frame_id', 'base_link').value
        self.odom_frame_id = self.node.declare_parameter('odom_frame_id', 'odom').value
        #giới hạn encoder
        self.encoder_min = self.node.declare_parameter('encoder_min', -32768).value
        self.encoder_max = self.node.declare_parameter('encoder_max', 32768).value
        #Ngưỡng cho việc quấn lại encoder khi giá trị encoder vượt qua giới hạn.
        self.encoder_low_wrap = self.node.declare_parameter('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min).value
        self.encoder_high_wrap = self.node.declare_parameter('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min).value

        self.t_delta = 1.0 / self.rate#Thời gian giữa các lần cập nhật
        self.t_next = self.node.get_clock().now() + self.t_delta#Thời điểm tiếp theo mà nút sẽ cập nhật dữ liệu

        # các biến nội bộ để lưu trữ dữ liệu và trạng thái của robot được khởi tạo
        self.enc_left = None # biến lưu trữ giá trị encoder từ lần đọc trước đó 
        self.enc_right = None
        self.left = 0  #giá trị thực tế trả về robot từ encorder được cập nahatsj
        self.right = 0
        self.lmult = 0#đếm số vòng quay đầy đủ encorder
        self.rmult = 0
        self.prev_lencoder = 0#giá trị encorder từ lần đọc trước đó
        self.prev_rencoder = 0
        self.x = 0  # position in xy plane 
        self.y = 0
        self.th = 0#tốc độ tuyến tính
        self.dr = 0#góc quay của robot
        self.then = self.node.get_clock().now()

        # Subscriptions
        self.lwheel_sub = self.node.create_subscription(Int64, "lwheel", self.lwheelCallback, 10)
        self.rwheel_sub = self.node.create_subscription(Int64, "rwheel", self.rwheelCallback, 10)
        self.odom_pub = self.node.create_publisher(Odometry, "/diff_tf_odom", 10)
        self.odom_broadcaster = TransformBroadcaster(self.node)

    def spin(self):
        while rclpy.ok():
            self.update()

    def update(self):
        now = self.node.get_clock().now()
        if now > self.t_next:#nếu thời gian hiện tại lớn hơn thời gian lần cuối nhận thì update time
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()
            #Tính toán giá trị thay đổi d của vị trí dựa trên giá trị của encoder bánh trái và bánh phải.
            if self.enc_left is None:
                d_left = 0
                d_right = 0
            else:
                # tính toán sự thay đổi về quãng đường di chuyển của bánh xe
                #quãng đường di chuyển(m) = (số xung mới -số xung cũ)/xung encorder mỗi mét)
                d_left = (self.left - self.enc_left) / self.left_ticks_meter
                d_right = (self.right - self.enc_right) / self.right_ticks_meter
                #update
            self.enc_left = self.left
            self.enc_right = self.right
            #Tính toán tốc độ tuyến tính (self.dx) và tốc độ góc (self.dr) dựa trên giá trị thay đổi d và góc quay th.
            d = (d_left + d_right) / 2  #mét
            th = (d_right - d_left) / self.base_width 
            self.dx = d / elapsed #m/s
            self.dr = th / elapsed # (rad/s) #sự chênh lệch v bánh phải và trái chia cho khoảng cách 2 bánh xe 
            #Tính toán và cập nhật vị trí của robot dựa trên tốc độ tuyến tính và góc quay.
           
            if d != 0:
                x = cos(th) * d
                y = -sin(th) * d
                self.x = self.x + (cos(self.th) * x - sin(self.th) * y)
                self.y = self.y + (sin(self.th) * x + cos(self.th) * y)
            if th != 0:
                self.th = self.th + th
            
            #Tạo một đối tượng Quaternion dựa trên góc quay của robot.
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin(self.th / 2)
            quaternion.w = cos(self.th / 2)
            #Tạo thông điệp Odometry và thiết lập các giá trị của nó.
            odom = Odometry()
            odom.header.stamp = now.to_msg()
            odom.header.frame_id = self.odom_frame_id
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.child_frame_id = self.base_frame_id
            odom.twist.twist.linear.x = self.dx
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = self.dr
            #Cài đặt giá trị cho ma trận covariance của Odometry.
            odom.pose.covariance[0] = 0.1
            odom.pose.covariance[7] = 0.1
            odom.pose.covariance[14] = 1000000.0
            odom.pose.covariance[21] = 1000000.0
            odom.pose.covariance[28] = 1000000.0
            odom.pose.covariance[35] = 0.3
            self.odom_pub.publish(odom)

            self.t_next = now + self.t_delta
            self.node.spin_once()

    def lwheelCallback(self, msg):
        enc = msg.data
        if enc < self.encoder_low_wrap and self.prev_lencoder > self.encoder_high_wrap:
            self.lmult = self.lmult + 1
        if enc > self.encoder_high_wrap and self.prev_lencoder < self.encoder_low_wrap:
            self.lmult = self.lmult - 1
        self.left = 1.0 * (enc + self.lmult * (self.encoder_max - self.encoder_min))
        self.prev_lencoder = enc

    def rwheelCallback(self, msg):
        enc = msg.data
        if enc < self.encoder_low_wrap and self.prev_rencoder > self.encoder_high_wrap:
            self.rmult = self.rmult + 1
        if enc > self.encoder_high_wrap and self.prev_rencoder < self.encoder_low_wrap:
            self.rmult = self.rmult - 1
        self.right = 1.0 * (enc + self.rmult * (self.encoder_max - self.encoder_min))
        self.prev_rencoder = enc


if __name__ == '__main__':
    try:
        diffTf = DiffTf()
        diffTf.spin()
    except rclpy.ROSInterruptException:
        pass

