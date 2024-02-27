#!/usr/bin/env python
"""
   pid_velocity - takes messages on wheel_vtarget 
      target velocities for the wheels and monitors wheel for feedback
      
    Copyright (C) 2012 Jon Stephan. 
     
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Float32
from numpy import array

class PidVelocity(Node):

    def __init__(self):
        super().__init__('pid_velocity')
        self.nodename = self.get_name()
        self.get_logger().info("%s started" % self.nodename)

        # Initialize variables
        self.target = 0
        self.motor = 0
        self.vel = 0
        self.integral = 0 #tích phân
        self.error = 0
        self.derivative = 0
        self.previous_error = 0 
        self.wheel_prev = 0
        self.wheel_latest = 0
        self.then = self.get_clock().now()
        self.wheel_mult = 0
        self.prev_encoder = 0

        # Get parameters
        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value
        self.out_min = self.get_parameter('out_min').value#đầu ra tối thiểu
        self.out_max = self.get_parameter('out_max').value#đầu ra tối đa
        self.rate = self.get_parameter('rate').value#giá trị tần suất
        self.rolling_pts = self.get_parameter('rolling_pts').value#điểm lăn
        self.timeout_ticks = self.get_parameter('timeout_ticks').value#điểm dấu tích time
        self.ticks_per_meter = self.get_parameter('ticks_meter').value#s/m
        self.vel_threshold = self.get_parameter('vel_threshold').value#ngưỡng vận tốc
        self.encoder_min = self.get_parameter('encoder_min').value
        self.encoder_max = self.get_parameter('encoder_max').value
        self.encoder_low_wrap = self.get_parameter('wheel_low_wrap').value
        self.encoder_high_wrap = self.get_parameter('wheel_high_wrap').value
        self.prev_vel = [0.0] * self.rolling_pts#điểm lăn tính từ điểm chạm coi là gốc tọa độ
        self.wheel_latest = 0.0
        self.prev_pid_time = self.get_clock().now()
        self.get_logger().debug("%s got Kp:%0.3f Ki:%0.3f Kd:%0.3f tpm:%0.3f" % (self.nodename, self.Kp, self.Ki, self.Kd, self.ticks_per_meter))

        # Subscribers/Publishers
        self.wheel_sub = self.create_subscription(Int64, 'wheel', self.wheelCallback, 10)
        self.target_sub = self.create_subscription(Float32, 'wheel_vtarget', self.targetCallback, 10)
        self.pub_motor = self.create_publisher(Float32, 'motor_cmd', 10)
        self.pub_vel = self.create_publisher(Float32, 'wheel_vel', 10)

    def spin(self):
        self.r = rclpy.create_rate(self.rate)
        self.then = self.get_clock().now()
        self.ticks_since_target = self.timeout_ticks
        self.wheel_prev = self.wheel_latest
        self.then = self.get_clock().now()

        while rclpy.ok():
            self.spinOnce()
            self.r.sleep()

    def spinOnce(self):
        self.previous_error = 0.0
        self.prev_vel = [0.0] * self.rolling_pts
        self.integral = 0.0#tích phân
        self.error = 0.0#sai số
        self.derivative = 0.0#đạo hàm
        self.vel = 0.0

        # Ktra xem có message vận tốc mới đc nhận không
        while rclpy.ok() and self.ticks_since_target < self.timeout_ticks:
            self.calcVelocity()
            self.doPid()
            self.pub_motor.publish(Float32(data=self.motor))
            self.r.sleep()
            self.ticks_since_target += 1#biến đếm
            if self.ticks_since_target == self.timeout_ticks:
                self.pub_motor.publish(Float32(data=0))

    def calcVelocity(self):#tính toán tốc độ hiện tại bánh xe và pub pub_vel
        self.dt_duration = self.get_clock().now() - self.then#thời gian giữa các lần tính toán
        self.dt = self.dt_duration.to_sec()
        self.get_logger().debug("-D- %s caclVelocity dt=%0.3f wheel_latest=%0.3f wheel_prev=%0.3f" %
                               (self.nodename, self.dt, self.wheel_latest, self.wheel_prev))
        #nếu k có sự thay đổi encorder
        if self.wheel_latest == self.wheel_prev:
            cur_vel = (1 / self.ticks_per_meter) / self.dt
            if abs(cur_vel) < self.vel_threshold:
                self.appendVel(0)
                self.calcRollingVel()
            else:
                if abs(cur_vel) < self.vel:
                    self.appendVel(cur_vel)
                    self.calcRollingVel()

        else:
            cur_vel = (self.wheel_latest - self.wheel_prev) / self.dt
            self.appendVel(cur_vel)
            self.calcRollingVel()
            self.get_logger().debug("-D- %s **** wheel updated vel=%0.3f **** " % (self.nodename, self.vel))
            self.wheel_prev = self.wheel_latest
            self.then = self.get_clock().now()

        self.pub_vel.publish(Float32(data=self.vel))
#Thêm giá trị tốc độ vào danh sách giá trị tốc độ trước đó.
    def appendVel(self, val):
        self.prev_vel.append(val)
        del self.prev_vel[0]
#Tính toán tốc độ trung bình dựa trên danh sách giá trị tốc độ.
    def calcRollingVel(self):
        p = array(self.prev_vel)
        self.vel = p.mean()
#Thực hiện điều khiển PID, tính toán giá trị động cơ dựa trên sai số, tích phân và đạo hàm.
    def doPid(self):
        pid_dt_duration = self.get_clock().now() - self.prev_pid_time#Thời gian giữa các lần tính toán PID.
        pid_dt = pid_dt_duration.to_sec()
        self.prev_pid_time = self.get_clock().now()

        self.error = self.target - self.vel#vận tốc mục tiêu (sp)-vẫn tốc tb
        self.integral = self.integral + (self.error * pid_dt)#tính tích phân
        self.derivative = (self.error - self.previous_error) / pid_dt#tính đạo hàm
        self.previous_error = self.error
     #công thức tìm sp 
        self.motor = (self.Kp * self.error) + (self.Ki * self.integral) + (self.Kd * self.derivative)

        if self.motor > self.out_max:
            self.motor = self.out_max
            self.integral = self.integral - (self.error * pid_dt)
        if self.motor < self.out_min:
            self.motor = self.out_min
            self.integral = self.integral - (self.error * pid_dt)

        if self.target == 0:
            self.motor = 0

        self.get_logger().debug("vel:%0.2f tar:%0.2f err:%0.2f int:%0.2f der:%0.2f ## motor:%d " %
                                (self.vel, self.target, self.error, self.integral, self.derivative, self.motor))
#xử lý dữ liệu encoder từ bánh xe.
    def wheelCallback(self, msg):
        enc = msg.data#giá trị encorder từ bánh xe phản hồi về
        if enc < self.encoder_low_wrap and self.prev_encoder > self.encoder_high_wrap:
            self.wheel_mult = self.wheel_mult + 1

        if enc > self.encoder_high_wrap and self.prev_encoder < self.encoder_low_wrap:
            self.wheel_mult = self.wheel_mult - 1

        self.wheel_latest = 1.0 * (enc + self.wheel_mult * (self.encoder_max - self.encoder_min)) / self.ticks_per_meter
        self.prev_encoder = enc#update
# xử lý giá trị target velocity.
    def targetCallback(self, msg):
        self.target = msg.data
        self.ticks_since_target = 0

def main(args=None):
    rclpy.init(args=args)
    pid_velocity = PidVelocity()
    pid_velocity.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
