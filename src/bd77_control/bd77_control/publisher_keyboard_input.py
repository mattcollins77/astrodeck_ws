import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import select
import tty
import termios
from bd77_common.msg import MoodMsg  # Replace with your actual package name


class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('publisher_keyboard_input')
        self.publisher_ = self.create_publisher(MoodMsg, 'mood_topic', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def publish_mood(self, mood, level, length):
        msg = MoodMsg()
        msg.mood = mood
        msg.level = level
        msg.length = length
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

    def timer_callback(self):
        if self.key_pressed():
            key = sys.stdin.read(1)
        
            # Create a new MoodMsg
            msg = MoodMsg()

            # Assign values based on the key pressed
            if key == 'h':
                msg.mood = "Happy"
                msg.level = 1
                msg.length = 1
            elif key == 's':
                msg.mood = "Sad"
                msg.level = 1
                msg.length = 1
            elif key == 'a':
                msg.mood = "Angry"
                msg.level = 1
                msg.length =1
            elif key == 'c':
                msg.mood = "Scared"
                msg.level = 1
                msg.length = 1
            elif key == 'e':
                msg.mood = "Elec"
                msg.level = 1
                msg.length = 1
            elif key == 'u':
                msg.mood = "Happy"
                msg.level = 2
                msg.length = 1
            elif key == 'i':
                msg.mood = "Sad"
                msg.level = 2
                msg.length = 1
            elif key == 'o':
                msg.mood = "Angry"
                msg.level = 2
                msg.length =1
            elif key == 'p':
                msg.mood = "Scared"
                msg.level = 2
                msg.length = 1
            
            else:
                # If an irrelevant key is pressed, return without publishing
                return

            # Publish the message
            self.publisher_.publish(msg)
            self.get_logger().info(f"Publishing Mood: {msg.mood}, Level: {msg.level}, Length: {msg.length}")


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
