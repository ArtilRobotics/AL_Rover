import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, UInt16
from motor_controller.msg import MotorState

class MotorSubscriber(Node):
    def __init__(self):
        super().__init__('motor_state_subscriber')
        self.subscription = self.create_subscription(
            MotorState,
            'motors/FL_lower/state',
            self.listener_callback,
            10
        )
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info(
            f'Torque: {msg.torque:.2f}, '
            f'Speed: {msg.speed:.2f}, '
            f'Position: {msg.position:.2f}, '
            f'Temperature: {msg.temperature:.2f}, '
            f'Error Code: {msg.error}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = MotorSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
