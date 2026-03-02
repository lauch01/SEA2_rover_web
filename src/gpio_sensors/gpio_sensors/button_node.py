import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import RPi.GPIO as GPIO

BUTTON_PIN = 17

class ButtonNode(Node):
    def __init__(self):
        super().__init__('button_node')

        self.publisher_ = self.create_publisher(Bool, 'button_state', 10)
        self.timer = self.create_timer(0.1, self.read_button)

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.get_logger().info('Nodo GPIO Button iniciado')

    def read_button(self):
        msg = Bool()
        # CORRECCIÓN: Convertimos el 0/1 a True/False
        msg.data = bool(GPIO.input(BUTTON_PIN)) 
        self.publisher_.publish(msg)

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()

def main():
    rclpy.init()
    node = ButtonNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
