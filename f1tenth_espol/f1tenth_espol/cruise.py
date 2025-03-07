#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sys
from ackermann_msgs.msg import AckermannDriveStamped

class TeleopPublisher(Node):
    def __init__(self, speed, steering_angle):
        super().__init__('override_ackermann_command')
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/teleop', 10)
        self.speed = speed
        self.steering_angle = steering_angle
        # Crea un timer que llama a publish_command a 10 Hz (0.1 s)
        self.timer = self.create_timer(0.1, self.publish_command)

    def publish_command(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = -self.speed
        msg.drive.steering_angle = self.steering_angle
        msg.drive.steering_angle_velocity = 0.0
        msg.drive.acceleration = 0.0
        msg.drive.jerk = 0.0

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: speed={self.speed:.2f}, steering_angle={self.steering_angle:.2f}')

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <speed> <steering_angle>")
        sys.exit(1)
    try:
        speed = float(sys.argv[1])
        steering_angle = float(sys.argv[2])
    except ValueError:
        print("Parameters must be floats.")
        sys.exit(1)

    node = TeleopPublisher(speed, steering_angle)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
