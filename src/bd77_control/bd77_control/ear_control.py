import rclpy
from rclpy.node import Node
import maestro

from bd77_common.msg import MoodMsg  # Replace with your actual package name
import serial


class EarControl(Node):

    def __init__(self):
        super().__init__('ear_control')
        self.mood_command_map = {
            ('Happy', 1, 1): 1,
            ('Sad', 2, 1): 2,
            # Add more mappings as needed
        }
        self.subscription = self.create_subscription(
            MoodMsg,
            'mood_topic',
            self.listener_callback,
            10)
      
        

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.mood)
        # Create a tuple from the incoming message parameters
        mood_key = (msg.mood, msg.level, msg.length)

        # Check if the tuple is in the mapping
        if mood_key in self.mood_command_map:
            command = self.mood_command_map[mood_key].encode()  # Encode the command
            self.communicate_with_teensy(command)
        else:
            self.get_logger().info('No command mapped for this mood combination')

    def communicate_with_maestro(self, command):
        try:
            # Open a connection to the Maestro controller
            servo = maestro.Controller('/dev/ttyACM1')

            # Run the script subroutine on the Maestro
          
            servo.runScriptSub(command)

            # Close the serial connection
            servo.close()
        except Exception as e:
            print(f"Error with Maestro communication: {e}")
            
    def __del__(self):
        if self.teensy_serial.is_open:
            self.teensy_serial.close()


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = EarControl()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()