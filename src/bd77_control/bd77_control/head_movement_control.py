import rclpy
from rclpy.node import Node
from . import maestro
from sensor_msgs.msg import Joy
from bd77_common.msg import MoodMsg  # Replace with your actual package name
import time
import random  # Import the random module
from threading import Timer

class HeadMovementControl(Node):

    def __init__(self):
        super().__init__('head_movement_control')
        self.joystick_paused = False
        self.random_mode_active = False
        self.last_non_random_mood = None
        self.override_timestamp = None  # Variable to store timestamp when overridden
        self.override_timer = None
        self.random_movement_timer = None

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
            ('Random', 1, 1): 99,
            ('Random', 0, 1): 100
        }
        self.servo = maestro.Controller('/dev/MyMaestro')
        self.servo.setSpeed(0, 0)
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
        command = self.mood_command_map.get(mood_key)

        self.get_logger().info('mood key: "%s"' % command)
        if command == 99:
            self.set_random_mode(True)
        elif command == 100:
            self.set_random_mode(False)
        else:
            if self.random_mode_active:
                self.suspend_random_mode()
            self.process_mood_command(mood_key)
    
    def set_random_mode(self, active):
        self.random_mode_active = active
        if active:
            self.start_random_movement()
        else:
            self.cancel_override_timer()
      
    def suspend_random_mode(self):
        if self.override_timer is not None:
            self.override_timer.cancel()
        self.override_timer = Timer(10, self.resume_random_mode)
        self.override_timer.start()

    def resume_random_mode(self):
        if not self.random_mode_active:
            self.set_random_mode(True)

    def cancel_override_timer(self):
        if self.override_timer is not None:
            self.override_timer.cancel()
            self.override_timer = None

    def start_random_movement(self):
        if not self.random_mode_active:
            return  # Stop random movements if random mode is deactivated

        # Implement random movement here
        self.get_logger().info('in random')
        # Generate random values within servo limits
        left_x = random.randint(5000, 7000)
        left_y = random.randint(4500, 5250)
        right_x = random.randint(4200, 5000)
        # Update servos with random targets
        self.servo.setTarget(2, left_x)
        self.servo.setTarget(5, left_y)
        self.servo.setTarget(3, right_x)

        if self.random_movement_timer:
            self.random_movement_timer.cancel()
        # Schedule next random movement after a random delay
        self.random_movement_timer = Timer(random.uniform(2.0, 5.0), self.start_random_movement)
        self.random_movement_timer.start()

    def joy_callback(self, msg):
        self.process_button_input(msg)

        self.process_joy_input(msg)
            
            
      

    def process_button_input(self, msg):

        if len(msg.buttons) > 1 and msg.buttons[0] == 1:
            # Button[1] is pressed
            self.publish_mood("Happy", 1, 1)
            if self.random_mode_active:
                self.get_logger().info('About to suspend random')
                self.suspend_random_mode()
            time.sleep(0.5)
        if len(msg.buttons) > 1 and msg.buttons[1] == 1:
            # Button[2] is pressed
            self.publish_mood("Sad", 1, 1)
            if self.random_mode_active:
                self.get_logger().info('About to suspend random')
                self.suspend_random_mode()
            time.sleep(0.5)
        if len(msg.buttons) > 1 and msg.buttons[2] == 1:
            # Button[3] is pressed
            self.publish_mood("Angry", 1, 1)
            if self.random_mode_active:
                self.get_logger().info('About to suspend random')
                self.suspend_random_mode()
            time.sleep(0.5)
        if len(msg.buttons) > 1 and msg.buttons[3] == 1:
            # Button[4] is pressed
            self.publish_mood("Scared", 1, 1)
            if self.random_mode_active:
                self.get_logger().info('About to suspend random')
                self.suspend_random_mode()
            time.sleep(0.5)

    def process_joy_input(self, msg):
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
            else:
                if self.random_mode_active:
                    self.get_logger().info('About to suspend random')
                    self.suspend_random_mode()
            if abs(joy_axis_value_left_y) <= dead_zone_threshold:
                joy_axis_value_left_y = 0.0
            else:
                if self.random_mode_active:
                    self.get_logger().info('About to suspend random')
                    self.suspend_random_mode()
            
            if abs(joy_axis_value_right_x) <= dead_zone_threshold:
                joy_axis_value_right_x = 0.0
            else:
                if self.random_mode_active:
                    self.get_logger().info('About to suspend random')
                    self.suspend_random_mode()
            if abs(joy_axis_value_right_y) <= dead_zone_threshold:
                joy_axis_value_right_y = 0.0
            else:
                if self.random_mode_active:
                    self.get_logger().info('About to suspend random')
                    self.suspend_random_mode()

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
