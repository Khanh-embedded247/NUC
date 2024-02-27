#!/usr/bin/env python
""" 
    wheel_loopback - simulates a wheel - just for testing
    
"""

#!/usr/bin/env python
#   Copyright 2012 Jon Stephan
#   jfstepha@gmail.com
#
#   This program is free software: you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation, either version 3 of the License, or
#   (at your option) any later version.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with this program.  If not, see <http://www.gnu.org/licenses/>.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64

class WheelLoopback(Node):
    def __init__(self):
        super().__init__('wheel_loopback')
        self.declare_parameter('rate', 200)
        self.declare_parameter('timeout_secs', 0.5)
        self.declare_parameter('ticks_meter', 50.0)
        self.declare_parameter('velocity_scale', 255.0)

        self.rate = self.get_parameter('rate').value
        self.timeout_secs = self.get_parameter('timeout_secs').value
        self.ticks_meter = self.get_parameter('ticks_meter').value
        self.velocity_scale = self.get_parameter('velocity_scale').value
        self.latest_motor = 0
        self.wheel = 0

        self.create_subscription(Int64, 'motor', self.motor_callback, 10)
        self.pub_wheel = self.create_publisher(Int64, 'wheel', 10)

    def spin(self):
        timer_rate = 1.0 / self.rate
        self.secs_since_target = self.timeout_secs
        self.then = self.get_clock().now()
        self.latest_msg_time = self.get_clock().now()
        self.get_logger().info("-D- spinning")

        while rclpy.ok():
            while rclpy.ok() and self.secs_since_target < self.timeout_secs:
                self.spin_once()
                self.secs_since_target = (self.get_clock().now() - self.latest_msg_time).to_sec()

            # It's been more than timeout_secs since we received a message
            self.secs_since_target = (self.get_clock().now() - self.latest_msg_time).to_sec()
            self.velocity = 0
            self.then = self.get_clock().now()
            self.get_logger().info("-D- outside: secs_since_target: %0.3f" % self.secs_since_target)

    def spin_once(self):
        self.velocity = self.latest_motor / self.velocity_scale
        if abs(self.velocity) > 0:
            self.seconds_per_tick = abs(1 / (self.velocity * self.ticks_meter))
            elapsed = (self.get_clock().now() - self.then).to_sec()
            self.get_logger().info("spinOnce: vel=%0.3f sec/tick=%0.3f elapsed:%0.3f" %
                                   (self.velocity, self.seconds_per_tick, elapsed))

            if elapsed > self.seconds_per_tick:
                self.get_logger().info("incrementing wheel")
                if self.velocity > 0:
                    self.wheel += 1
                else:
                    self.wheel -= 1
                self.pub_wheel.publish(Int64(data=self.wheel))
                self.then = self.get_clock().now()

    def motor_callback(self, msg):
        self.latest_motor = msg.data
        self.latest_msg_time = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    wheel_loopback = WheelLoopback()
    wheel_loopback.spin()
    rclpy.spin(wheel_loopback)
    wheel_loopback.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
