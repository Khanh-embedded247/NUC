#!/usr/bin/env python
"""
   twist_to_motors - converts a twist message to motor commands.  Needed for navigation stack
   
   
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
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class TwistToMotors(Node):

    def __init__(self):
        super().__init__('twist_to_motors')
        nodename = self.get_name()
        self.get_logger().info("%s started" % nodename)

        self.w = self.get_parameter('base_width').value

        self.pub_lmotor = self.create_publisher(Float32, 'lwheel_vtarget', 10)
        self.pub_rmotor = self.create_publisher(Float32, 'rwheel_vtarget', 10)
        self.sub_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self.twistCallback, 10)

        self.rate = self.get_parameter('rate').value
        self.timeout_ticks = self.get_parameter('timeout_ticks').value
        self.left = 0
        self.right = 0
        self.dx = 0
        self.dr = 0
        self.dy = 0

    def spin(self):
        r = rclpy.create_rate(self.rate)
        idle = rclpy.create_rate(10)
        self.then = self.get_clock().now()
        self.ticks_since_target = self.timeout_ticks

        while rclpy.ok():
            while rclpy.ok() and self.ticks_since_target < self.timeout_ticks:
                self.spinOnce()
                r.sleep()
            idle.sleep()

    def spinOnce(self):
        self.right = 1.0 * self.dx + self.dr * self.w / 2
        self.left = 1.0 * self.dx - self.dr * self.w / 2

        self.pub_lmotor.publish(Float32(data=self.left))
        self.pub_rmotor.publish(Float32(data=self.right))

        self.ticks_since_target += 1

    def twistCallback(self, msg):
        self.ticks_since_target = 0
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
