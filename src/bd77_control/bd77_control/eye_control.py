import rclpy
from rclpy.node import Node

from bd77_common.msg import MoodMsg  # Replace with your actual package name
import serial


class EyeControl(Node):

    def __init__(self):
        super().__init__('eye_control')
        self.mood_command_map = {
            ('Happy', 1, 1): '<SH0>',
            ('Sad', 1, 1): '<SS0>',
            ('Scared', 1, 1): '<SC0>',
            ('Angry', 1, 1): '<SM0>',
            ('Happy', 2, 1): '<SH1>',
            ('Sad', 2, 1): '<SS1>',
            ('Scared', 2, 1): '<SC1>',
            ('Angry', 2, 1): '<SM1>',
            ('Elec', 1, 1): '<SE>',
            # Add more mappings as needed
        }
        self.subscription = self.create_subscription(
            MoodMsg,
            'mood_topic',
            self.listener_callback,
            10)
        self.serial_port_teensy = '/dev/MyESP32'
        self.baudrate = 9600
        try:
            self.teensy_serial = serial.Serial(self.serial_port_teensy, self.baudrate, timeout=1)
        except serial.SerialException as e:
            self.teensy_serial = None
            self.get_logger().error(f"Failed to open serial port: {e}")

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

    def communicate_with_teensy(self, command):
        if self.teensy_serial is not None:
            try:
                self.teensy_serial.write(command)
                self.get_logger().info('Command sent to Teensy: %s' % command)
            except Exception as e:
                self.get_logger().error('Error with Teensy communication: %s' % str(e))
        else:
            self.get_logger().error('Teensy serial connection not established')
            
    def __del__(self):
        if self.teensy_serial.is_open:
            self.teensy_serial.close()


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = EyeControl()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()