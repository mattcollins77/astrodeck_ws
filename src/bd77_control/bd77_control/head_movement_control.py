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
            # Map the normalized value to the servo position range
            servo_position = int(self.map_value(normalized_value, -1, 1, 100, 9000))  # Assuming 3000-9000 is the servo range
            # Send the servo position command to the Maestro
            self.servo.setTarget(0, servo_position)

    def map_value(self, value, in_min, in_max, out_min, out_max):
        # Map a value from one range to another
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


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
