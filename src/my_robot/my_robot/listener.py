from std_msgs.msg import String
import rclpy
from rclpy.node import Node

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.sub = self.create_subscription(
            String,
            'hello',
            self.callback,
            10)

    def callback(self, msg):
        self.get_logger().info(f'Recibido: {msg.data}')

def main():
    rclpy.init()
    node = Listener()
    rclpy.spin(node)
    rclpy.shutdown()
