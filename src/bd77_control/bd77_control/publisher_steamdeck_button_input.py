import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import select
import tty
import termios

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('publisher_steamdeck_button')
        self.publisher_ = self.create_publisher(String, 'mood', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def timer_callback(self):
        if self.key_pressed():
            key = sys.stdin.read(1)
            if key in ['a', 'b', 'c', 'd']:
                msg = String()
                msg.data = f"Key '{key}' pressed"
                self.publisher_.publish(msg)
                self.get_logger().info(f"Publishing: '{msg.data}'")

    def key_pressed(self):
        return select.select([sys.stdin], [], [], 0)[0] != []

    def __del__(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_attr)

def main(args=None):
    rclpy.init(args=args)
    keyboard_publisher = KeyboardPublisher()
    rclpy.spin(keyboard_publisher)
    keyboard_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
