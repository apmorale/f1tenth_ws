import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        # Suscribirse al tópico de `teleop_keyboard`
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',  # Tópico estándar de teleop_keyboard
            self.keyboard_callback,
            10
        )
        # Publicador para enviar comandos al carro
        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            '/teleop',
            10
        )
        self.speed_scale = 5.0  # Escalado de velocidad
        self.steering_scale = 0.34  # Escalado de dirección
        self.get_logger().info("KeyboardTeleop node started")

    def keyboard_callback(self, twist_msg):
        # Convertir los comandos de Twist en AckermannDriveStamped
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = twist_msg.linear.x * self.speed_scale
        drive_msg.drive.steering_angle = twist_msg.angular.z * self.steering_scale

        # Publicar el mensaje de control
        self.publisher.publish(drive_msg)

        # Log para monitoreo
        self.get_logger().info(f"Speed: {drive_msg.drive.speed:.2f}, Steering Angle: {drive_msg.drive.steering_angle:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
