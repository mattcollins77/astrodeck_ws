import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from bd77_common.msg import MoodMsg  # Replace with your actual package name

class SteamdeckInput(Node):

    def __init__(self):
        super().__init__('steamdeck_input_subscriber')
        
        self.subscription = self.create_subscription(
            Joy,
            'steamdeckjoy',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(MoodMsg, 'mood_topic', 10)

    def listener_callback(self, msg):
        if len(msg.buttons) > 1 and msg.buttons[1] == 1:
            # Button[1] is pressed
            self.publish_mood("Happy", 1, 1)

    def publish_mood(self, mood, level, length):
        msg = MoodMsg()
        msg.mood = mood
        msg.level = level
        msg.length = length
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing Mood: "%s", Level: %d, Length: %d' % (msg.mood, msg.level, msg.length))

def main(args=None):
    rclpy.init(args=args)
    steamdeck_input_subscriber_node = SteamdeckInput()
    rclpy.spin(steamdeck_input_subscriber_node)
    steamdeck_input_subscriber_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
