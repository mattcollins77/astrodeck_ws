import maestro
import time

# Define the serial port (update with your correct port)
serial_port = '/dev/ttyACM0'  # Use 'ls /dev/tty*' to find the correct port

# Open a connection to the Maestro controller
servo = maestro.Controller(serial_port)

# Set the acceleration and target positions for servo 0 and servo 1


# Run the script subroutine on the Maestro (substitute 0 with the desired subroutine number)
subroutine_number = 1
servo.runScriptSub(subroutine_number)

# Close the serial connection when done
servo.close()
