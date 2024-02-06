import rclpy
from rclpy.node import Node
from . import maestro
from sensor_msgs.msg import Joy
from bd77_common.msg import MoodMsg  # Replace with your actual package name
import serial
import time
import random  # Import the random module

class HeadMovementControl(Node):

    def __init__(self):
        super().__init__('head_movement_control')
        self.joystick_paused = False
        self.timer = self.create_timer(4.0, self.unpause_joystick)
        self.random_mode_active = False
        self.last_non_random_mood = None
        self.override_timestamp = None  # Variable to store timestamp when overridden

        self.mood_command_map = {
            ('Happy', 1, 1): 3,
            ('Sad', 1, 1): 5,
            ('Scared', 1, 1): 4,
            ('Angry', 1, 1): 4,
            ('Happy', 2, 1): 2,
            ('Sad', 2, 1): 5,
            ('Scared', 2, 1): 4,
            ('Angry', 2, 1): 4,
            ('Elec', 1, 1): 6,
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
        
        self.publisher_ = self.create_publisher(MoodMsg, 'mood_topic', 10)
      
    def mood_callback(self, msg):
        self.get_logger().info('I heard on mood_topic: "%s"' % msg.mood)
        mood_key = (msg.mood, msg.level, msg.length)
        current_time = time.time()

        if mood_key == ('Random', 1, 1):  # Activate random mode
            self.random_mode_active = True
            self.last_non_random_mood = None
        elif mood_key == ('Random', 0, 1):  # Deactivate random mode
            self.random_mode_active = False
            if self.last_non_random_mood:
                # Restore the last non-random mood if available
                self.process_mood_command(self.last_non_random_mood)
        else:
            self.process_mood_command(mood_key)
            # Store the last non-random mood
            if mood_key != ('Random', 1, 1):
                self.last_non_random_mood = mood_key
            self.random_mode_active = False
            self.override_timestamp = current_time

    def joy_callback(self, msg):
        current_time = time.time()
        if self.random_mode_active:
            if not self.override_timestamp:
                self.override_timestamp = current_time
            elif current_time - self.override_timestamp >= 20:
                self.override_timestamp = None
                self.random_mode_active = False

        # Handle random movements within servo limits
        if self.random_mode_active:
            # Generate random values within a smaller range for servos 2 and 5
            left_x = random.randint(5000, 7000)
            left_y = random.randint(4500, 5250)
            right_x = random.randint(5000, 7000)

            # Set the random values to servos
            self.servo.setTarget(2, left_x)
            self.servo.setTarget(5, left_y)
            self.servo.setTarget(3, right_x)

            # Introduce a delay between random movements (adjust as needed)
            time.sleep(random.uniform(2.0, 5.0))  # Random delay between 2 and 5 seconds

        # Add your Steam Deck Joy callback logic here
        if len(msg.buttons) > 1 and msg.buttons[0] == 1:
            # Button[1] is pressed
            self.publish_mood("Happy", 1, 1)
            self.override_timestamp = current_time  # Reset the override timestamp
            time.sleep(0.5)
        if len(msg.buttons) > 1 and msg.buttons[1] == 1:
            # Button[2] is pressed
            self.publish_mood("Sad", 1, 1)
            self.override_timestamp = current_time  # Reset the override timestamp
            time.sleep(0.5)
        if len(msg.buttons) > 1 and msg.buttons[2] == 1:
            # Button[3] is pressed
            self.publish_mood("Angry", 1, 1)
            self.override_timestamp = current_time  # Reset the override timestamp
            time.sleep(0.5)
        if len(msg.buttons) > 1 and msg.buttons[3] == 1:
            # Button[4] is pressed
            self.publish_mood("Scared", 1, 1)
            self.override_timestamp = current_time  # Reset the override timestamp
            time.sleep(0.5)

        if self.joystick_paused:
            return
        if len(msg.axes) > 0:
            joy_axis_value_left_x = msg.axes[0]
            joy_axis_value_left_y = msg.axes[1]
            joy_axis_value_right_x = msg.axes[3]
            joy_axis_value_right_y = msg.axes[4]

            # Define dead zone thresholds (adjust as needed)
            dead_zone_threshold = 0.1  # Adjust this threshold as needed

            # Apply centering correction if joystick values are close to center
            if abs(joy_axis_value_left_x) <= dead_zone_threshold:
                joy_axis_value_left_x = 0.0
            if abs(joy_axis_value_left_y) <= dead_zone_threshold:
                joy_axis_value_left_y = 0.0
            if abs(joy_axis_value_right_x) <= dead_zone_threshold:
                joy_axis_value_right_x = 0.0
            if abs(joy_axis_value_right_y) <= dead_zone_threshold:
                joy_axis_value_right_y = 0.0

            # Perform joystick processing here
            left_x = int(self.map_value(max(min(joy_axis_value_left_x, 1.0), -1.0), -1, 1, 8444, 4032))
            left_y = int(self.map_value(max(min(joy_axis_value_left_y, 1.0), -1.0), -1, 1, 5776, 3964))
            right_x = int(self.map_value(max(min(joy_axis_value_right_x, 1.0), -1.0), -1, 1, 5224, 3964))
            right_y = int(self.map_value(max(min(joy_axis_value_right_y, 1.0), -1.0), -1, 1, 5776, 3964))

            self.servo.setTarget(2, left_x)
            self.servo.setTarget(5, left_y)
            self.servo.setTarget(3, right_x)


    def map_value(self, value, from_low, from_high, to_low, to_high):
    # Map 'value' from the range [from_low, from_high] to [to_low, to_high]
        return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low
    
    def unpause_joystick(self):
        # Timer callback to unpause joystick input
        self.joystick_paused = False
        self.get_logger().info('Joystick input unpaused')
    
    def publish_mood(self, mood, level, length):
        msg = MoodMsg()
        msg.mood = mood
        msg.level = level
        msg.length = length
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing Mood: "%s", Level: %d, Length: %d' % (msg.mood, msg.level, msg.length))
        

    def process_mood_command(self, mood_key):
        if mood_key in self.mood_command_map:
            command = self.mood_command_map[mood_key]
            self.joystick_paused = True
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
