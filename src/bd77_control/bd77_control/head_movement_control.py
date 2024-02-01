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
        self.servo = maestro.Controller('/dev/MyMaestro')
        self.servo.setSpeed(0,0)
        self.mood_subscription = self.create_subscription(
            MoodMsg,
            'mood_topic',
            self.mood_callback,
            10)

        self.joy_subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
      
    def mood_callback(self, msg):
        self.get_logger().info('I heard on mood_topic: "%s"' % msg.mood)
        mood_key = (msg.mood, msg.level, msg.length)
        self.process_mood_command(mood_key)

    def joy_callback(self, msg):
        # Add your Steam Deck Joy callback logic here
        if len(msg.buttons) > 1 and msg.buttons[1] == 1:
            # Button[1] is pressed
            self.publish_mood("Happy", 1, 1)
        if len(msg.axes) > 0:
            joy_axis_value_left_x = msg.axes[0]
            left_x = int(self.map_value(max(min(joy_axis_value_left_x, 1.0), -1.0), -1, 1, 4032, 8444))
            
            joy_axis_value_left_y = msg.axes[1]
            left_y = int(self.map_value(max(min(joy_axis_value_left_y, 1.0), -1.0), -1, 1, 3964, 5776))
            
            joy_axis_value_right_x = msg.axes[0]
            right_x = int(self.map_value(max(min(joy_axis_value_right_x, 1.0), -1.0), -1, 1, 3964, 5224))
            
            joy_axis_value_right_y = msg.axes[1]
            right_y = int(self.map_value(max(min(joy_axis_value_right_y, 1.0), -1.0), -1, 1, 3964, 5776))
            
            self.servo.setTarget(2, left_x) 
            self.servo.setTarget(5, left_y)  
            self.servo.setTarget(3, right_x)
         

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
