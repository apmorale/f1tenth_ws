import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
import sys
import termios
import tty

# Clase para manejar las entradas del teclado y controlar el robot
class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_node')
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/keyboard', 10)
        self.speed = 0.0
        self.steering_angle = 0.0
        self.speed_increment = 0.1
        self.steering_increment = 0.05
        self.get_logger().info("Nodo inicializado. Usa W/A/S/D para controlar el robot.")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()

                if key == 'w':
                    self.speed += self.speed_increment
                elif key == 's':
                    self.speed -= self.speed_increment
                elif key == 'a':
                    self.steering_angle += self.steering_increment
                elif key == 'd':
                    self.steering_angle -= self.steering_increment
                elif key == '\x03':  # Ctrl+C
                    break

                drive_msg = AckermannDriveStamped()
                drive_msg.drive.speed = self.speed
                drive_msg.drive.steering_angle = self.steering_angle
                self.publisher_.publish(drive_msg)

                self.get_logger().info(f"Velocidad: {self.speed:.2f} m/s, Ángulo de dirección: {self.steering_angle:.2f} rad")

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

        finally:
            # Detener el robot antes de cerrar
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = 0.0
            drive_msg.drive.steering_angle = 0.0
            self.publisher_.publish(drive_msg)
            self.get_logger().info("Nodo finalizado. Robot detenido.")

# Configuración inicial del terminal para leer teclas
settings = termios.tcgetattr(sys.stdin)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

