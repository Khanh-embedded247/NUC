#!/usr/bin/env python

"""
   wheel_scaler
   scales the wheel readings (and inverts the sign)
   
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
from std_msgs.msg import Int64

class WheelScaler(Node):
    def __init__(self):
        super().__init__('wheel_scaler')
        self.declare_parameter('distance_scale', 1.0)
        self.scale = self.get_parameter('distance_scale').value

        self.lscaled_pub = self.create_publisher(Int64, 'lwheel_scaled', 10)
        self.rscaled_pub = self.create_publisher(Int64, 'rwheel_scaled', 10)

        self.create_subscription(Int64, 'lwheel', self.lwheel_callback, 10)
        self.create_subscription(Int64, 'rwheel', self.rwheel_callback, 10)

        self.get_logger().info("wheel_scaler started with scale: %0.2f" % self.scale)

    def lwheel_callback(self, msg):
        self.lscaled_pub.publish(Int64(data=msg.data * -1 * self.scale))

    def rwheel_callback(self, msg):
        self.rscaled_pub.publish(Int64(data=msg.data * -1 * self.scale))


def main(args=None):
    rclpy.init(args=args)
    wheel_scaler = WheelScaler()
    rclpy.spin(wheel_scaler)
    wheel_scaler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
