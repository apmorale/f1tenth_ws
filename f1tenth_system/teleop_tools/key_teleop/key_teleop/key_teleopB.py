#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2013 PAL Robotics SL.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of PAL Robotics SL. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#
# Authors:
#   * Siegfried-A. Gevatter
#   * Jeremie Deray (artivis)

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
import sys
import tty
import termios
import threading

class KeyTeleopBNode(Node):
    def __init__(self):
        super().__init__('key_teleop_b')
        self.publisher = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10)

        self.speed = 0.0
        self.steering_angle = 0.0
        self.speed_increment = 0.1
        self.steering_increment = 0.1

        self.keep_publishing = True
        self.thread = threading.Thread(target=self.publish_loop)
        self.thread.start()

        self.get_logger().info("Teleoperation node B started. Use W/S to control speed and A/D to control steering.")

    def run(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while True:
                key = sys.stdin.read(1)
                if key == '\x03':  # Ctrl+C to exit
                    break
                elif key.lower() == 'w':
                    self.speed += self.speed_increment
                elif key.lower() == 's':
                    self.speed -= self.speed_increment
                elif key.lower() == 'a':
                    self.steering_angle += self.steering_increment
                elif key.lower() == 'd':
                    self.steering_angle -= self.steering_increment
                elif key.lower() == ' ':
                    self.speed = 0.0
                    self.steering_angle = 0.0

                # Log the current command
                self.get_logger().info(f"Updated Command - Speed: {self.speed:.2f}, Steering Angle: {self.steering_angle:.2f}")

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            self.keep_publishing = False
            self.thread.join()

    def publish_loop(self):
        rate = self.create_rate(10)  # Publish at 10 Hz
        while self.keep_publishing:
            self.publish_command()
            rate.sleep()

    def publish_command(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = self.speed
        msg.drive.steering_angle = self.steering_angle
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = KeyTeleopBNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

