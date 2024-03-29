import rclpy
from rclpy.node import Node
from . import maestro

from bd77_common.msg import MoodMsg  # Replace with your actual package name
import serial


class EarControl(Node):

    def __init__(self):
        super().__init__('ear_control')
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
            command = self.mood_command_map[mood_key]  # Encode the command
            self.communicate_with_maestro(command)
        else:
            self.get_logger().info('No command mapped for this mood combination')

    def communicate_with_maestro(self, command):
        try:
            # Open a connection to the Maestro controller
            servo = maestro.Controller('/dev/MyMaestro')

            # Run the script subroutine on the Maestro
          
            servo.runScriptSub(command)

            # Close the serial connection
            servo.close()
        except Exception as e:
            self.get_logger().info('Error with Maestro')
            
    

def main(args=None):
    rclpy.init(args=args)

    ear_control_node = EarControl()

    rclpy.spin(ear_control_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ear_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()