#!/usr/bin/env python
"""
   twist_to_motors - converts a twist message to motor commands.  Needed for navigation stack
   
 
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from simple_pid import PID

DIAMETER = 0.17  # đường kính bánh xe


class TwistToMotors(Node):

    def __init__(self):
        super().__init__('twist_to_motors')
        nodename = self.get_name()
        self.get_logger().info("%s started" % nodename)

        self.w = self.get_parameter('base_width').value#độ rộng cơ sở của robot
        #publisher các lệnh vận tốc cho bánh trái và phải
        self.pub_lmotor = self.create_publisher(Float64, 'lwheel_vtarget', 10)
        self.pub_rmotor = self.create_publisher(Float64, 'rwheel_vtarget', 10)
        #subscribers để nhận thông điệp vận tốc (cmd_vel), tốc độ bánh xe phải (rspeed), và tốc độ bánh xe trái (lspeed).
        self.sub_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self.twistCallback, 10)
        self.sub_rspeed = self.create_subscription(Float64, 'rspeed', self.rightWheelVelocityCallback, 10)
        self.sub_lspeed = self.create_subscription(Float64, 'lspeed', self.leftWheelVelocityCallback, 10)
        #tần số vòng lặp
        self.rate = self.get_parameter('rate').value
        #Số ticks tối đa được phép từ khi nhận thông điệp tốc độ.
        self.timeout_ticks = self.get_parameter('timeout_ticks').value
        #Số ticks mỗi mét cho bánh xe trái và phải.
        self.ticks_meter_l = self.get_parameter('ticks_meter_left').value
        self.ticks_meter_r = self.get_parameter('ticks_meter_right').value
        #hệ số đk PID
        self.Kp = self.get_parameter('Kp').value
        
        #giá trị đk cho bánh xe trái và phải
        self.left = 0
        self.right = 0
        #Giá trị mục tiêu vận tốc cho bánh xe trái và phải.
        self.left_target = 0
        self.right_target = 0
        #Vận tốc thực tế của bánh xe trái và phải
        self.wheelVelo_l = 0
        self.wheelVelo_r = 0
        #Các bộ điều khiển PID cho bánh xe trái và phải.
        self.pid_l = PID(0.6, 0.5, 0.0, setpoint=1)
        self.pid_r = PID(0.5, 0.4, 0.0, setpoint=1)

        self.pid_l.sample_time = 0.01
        self.pid_r.sample_time = 0.01

        self.pid_l.output_limits = (-1, 1)
        self.pid_r.output_limits = (-1, 1)

    def rightWheelVelocityCallback(self, msg):
        self.wheelVelo_r = msg.data / 2 * DIAMETER  # m/s

    def leftWheelVelocityCallback(self, msg):
        self.wheelVelo_l = msg.data / 2 * DIAMETER  # m/s

    def spin(self):
        r = rclpy.create_rate(self.rate)
        idle = rclpy.create_rate(100)
        self.then = self.get_clock().now()
        self.ticks_since_target = self.timeout_ticks

        while rclpy.ok():
            while rclpy.ok() and self.ticks_since_target < self.timeout_ticks:
                self.spinOnce()
                r.sleep()
            self.doPID()
            idle.sleep()

    def spinOnce(self):#Giá trị tuyến tính và góc vòng quay từ thông điệp Twist.
        self.right_target = 1.0 * self.dx + self.dr * self.w / 2
        self.left_target = 1.0 * self.dx - self.dr * self.w / 2

        self.pub_lmotor.publish(Float64(data=self.left_target))
        self.pub_rmotor.publish(Float64(data=self.right_target))
        self.ticks_since_target += 1

    def doPID(self):
        SETPOINT_MUL = 1#hệ số điều chỉnh gtri setpoint của PID
        self.pid_l.setpoint = self.left_target * SETPOINT_MUL
        self.pid_r.setpoint = self.right_target * SETPOINT_MUL
        #Tính toán giá trị điều khiển từ bộ điều khiển PID cho bánh xe trái và phải.
        self.left = self.pid_l(self.wheelVelo_l)
        self.right = self.pid_r(self.wheelVelo_r)

        if (self.left_target == 0.0) and (self.right_target == 0.0):
            self.pub_lmotor.publish(Float64(data=self.left_target))
            self.pub_rmotor.publish(Float64(data=self.right_target))

        self.pub_lmotor.publish(Float64(data=self.left))
        self.pub_rmotor.publish(Float64(data=self.right))

    def twistCallback(self, msg):
        # Lấy giá trị tuyến tính và góc vòng quay từ thông điệp Twist.
        self.ticks_since_target = 0
        #Cập nhật giá trị tuyến tính, góc vòng quay, và hướng y.
        self.dx = msg.linear.x
        self.dr = msg.angular.z
        self.dy = msg.linear.y


def main(args=None):
    rclpy.init(args=args)
    twist_to_motors = TwistToMotors()
    twist_to_motors.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
