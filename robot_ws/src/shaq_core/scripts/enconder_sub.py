import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class EncoderSubscriber(Node):
    def __init__(self):
        super().__init__('encoder_subscriber')
        self.subscription = self.create_subscription(
            Twist, '/debug/move_encoder', self.listener_callback, 10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'Received encoder data: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = EncoderSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
