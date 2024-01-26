import rclpy
from rclpy.node import Node
from . import maestro
from sensor_msgs.msg import Joy
from bd77_common.msg import MoodMsg  # Replace with your actual package name
import serial

class HeadMovementControl(Node):

    def __init__(self):
        super().__init__('head_movement_control')
        self.mood_command_map = {
            ('Happy', 1, 1): 0,
            ('Sad', 1, 1): 1,
            ('Scared', 1, 1): 2,
            ('Angry', 1, 1): 3,
            ('Happy', 2, 1): 4,
            ('Sad', 2, 1): 5,
            ('Scared', 2, 1): 6,
            ('Angry', 2, 1): 7,
            ('Elec', 1, 1): 8,
            # Add more mappings as needed
        }
        self.servo = maestro.Controller('/dev/ttyACM1')
        self.mood_subscription = self.create_subscription(
            MoodMsg,
            'mood_topic',
            self.mood_callback,
            10)

        self.joy_subscription = self.create_subscription(
            Joy,
            'steamdeckjoy',
            self.joy_callback,
            10)
      
    def mood_callback(self, msg):
        self.get_logger().info('I heard on mood_topic: "%s"' % msg.mood)
        mood_key = (msg.mood, msg.level, msg.length)
        self.process_mood_command(mood_key)

    def joy_callback(self, msg):
        self.get_logger().info('I heard on steamdeckjoy: "%s"' % msg)
        # Add your Steam Deck Joy callback logic here
        if len(msg.axes) > 0:
            joy_axis_value = msg.axes[0]
    # Normalize the joystick value to the range -1 to 1
            normalized_value = max(min(joy_axis_value, 1.0), -1.0)
    
            blah = int(self.map_value(normalized_value, -1, 1, 1000, 2000))
            self.get_logger().info('I heard on steamdeckjoy: "%s"' % blah)
            # Normalize the joystick value to the range -1 to 1
            self.servo.setTarget(0, blah)  # Assuming 0-1 range maps to 0-6000 servo position

    def map_value(self, value, from_low, from_high, to_low, to_high):
    # Map 'value' from the range [from_low, from_high] to [to_low, to_high]
        return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low

    def process_mood_command(self, mood_key):
        if mood_key in self.mood_command_map:
            command = self.mood_command_map[mood_key]
            self.communicate_with_maestro(command)
        else:
            self.get_logger().info('No command mapped for this mood combination')

    def communicate_with_maestro(self, command):
        try:
            
            self.servo.runScriptSub(command)
            
        except Exception as e:
            self.get_logger().info('Error with Maestro')

def main(args=None):
    rclpy.init(args=args)
    head_movement_control_node = HeadMovementControl()
    rclpy.spin(head_movement_control_node)
    head_movement_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
