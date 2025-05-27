import rclpy
from rclpy.node import Node
from motor_controller.msg import MotorCommand

class MotorCommandPublisher(Node):
    def __init__(self):
        super().__init__('motor_command_publisher')

        self.publisher = self.create_publisher(
            MotorCommand,
            'motors/FL_lower/cmd',
            10
        )

        self.timer = self.create_timer(1.0, self.publish_command)

    def publish_command(self):
        msg = MotorCommand()
        msg.torque_desired = 0.0
        msg.speed_desired = 0.0
        msg.position_desired = 0.0
        msg.k_pos = 0.0
        msg.k_spd = 0.0

        self.publisher.publish(msg)
        self.get_logger().info(f'Mensaje Enviado: {msg}')

def main(args=None):
    rclpy.init(args=args)
    node = MotorCommandPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
