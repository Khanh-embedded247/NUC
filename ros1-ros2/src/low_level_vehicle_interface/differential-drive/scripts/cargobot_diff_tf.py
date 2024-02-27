#!/usr/bin/env python

"""
   diff_tf.py - follows the output of a wheel encoder and
   creates tf and odometry messages.
   some code borrowed from the arbotix diff_controller script
   A good reference: http://rossum.sourceforge.net/papers/DiffSteer/
   
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
   
   ----------------------------------
   Portions of this code borrowed from the arbotix_python diff_controller.
   
diff_controller.py - controller for a differential drive
  Copyright (c) 2010-2011 Vanadium Labs LLC.  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of Vanadium Labs LLC nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""

import rclpy
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64
from tf2_ros import TransformBroadcaster
from math import sin, cos, pi

class DiffTf:

    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("diff_tf")
        self.nodename = self.node.get_name()
        self.node.get_logger().info("-I- %s started" % self.nodename)

        # Parameters
        self.rate = self.node.declare_parameter('rate', 10.0).value
        self.left_ticks_meter = self.node.declare_parameter('left_ticks_meter', 50).value
        self.right_ticks_meter = self.node.declare_parameter('right_ticks_meter', 50).value
        self.base_width = self.node.declare_parameter('base_width', 0.35).value
        self.base_frame_id = self.node.declare_parameter('base_frame_id', 'base_link').value
        self.odom_frame_id = self.node.declare_parameter('odom_frame_id', 'odom').value
        self.encoder_min = self.node.declare_parameter('encoder_min', -32768).value
        self.encoder_max = self.node.declare_parameter('encoder_max', 32768).value
        self.encoder_low_wrap = self.node.declare_parameter('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min).value
        self.encoder_high_wrap = self.node.declare_parameter('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min).value

        self.t_delta = 1.0 / self.rate
        self.t_next = self.node.get_clock().now() + self.t_delta

        # Internal data
        self.enc_left = None
        self.enc_right = None
        self.left = 0
        self.right = 0
        self.lmult = 0
        self.rmult = 0
        self.prev_lencoder = 0
        self.prev_rencoder = 0
        self.x = 0
        self.y = 0
        self.th = 0
        self.dx = 0
        self.dr = 0
        self.then = self.node.get_clock().now()

        # Subscriptions
        self.lwheel_sub = self.node.create_subscription(Int64, "rlwheel", self.lwheelCallback, 10)
        self.rwheel_sub = self.node.create_subscription(Int64, "rrwheel", self.rwheelCallback, 10)
        self.odom_pub = self.node.create_publisher(Odometry, "odom", 10)
        self.odom_broadcaster = TransformBroadcaster(self.node)

    def spin(self):
        while rclpy.ok():
            self.update()

    def update(self):
        now = self.node.get_clock().now()
        if now > self.t_next:
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()

            if self.enc_left is None:
                d_left = 0
                d_right = 0
            else:
                d_left = (self.left - self.enc_left) / self.left_ticks_meter
                d_right = (self.right - self.enc_right) / self.right_ticks_meter
            self.enc_left = self.left
            self.enc_right = self.right

            d = (d_left + d_right) / 2
            th = (d_right - d_left) / self.base_width
            self.dx = d / elapsed
            self.dr = th / elapsed

            if d != 0:
                x = cos(th) * d
                y = -sin(th) * d
                self.x = self.x + (cos(self.th) * x - sin(self.th) * y)
                self.y = self.y + (sin(self.th) * x + cos(self.th) * y)
            if th != 0:
                self.th = self.th + th

            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin(self.th / 2)
            quaternion.w = cos(self.th / 2)

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
