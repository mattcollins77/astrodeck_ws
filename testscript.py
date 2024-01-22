import maestro
import time
import serial

# Define the serial port (update with your correct port)
serial_port = '/dev/ttyACM1'  # Use 'ls /dev/tty*' to find the correct port
teensy_port = '/dev/ttyACM0'
baudrate = 9600

# Open a connection to the Maestro controller
servo = maestro.Controller(serial_port)

# Set the acceleration and target positions for servo 0 and servo 1


# Run the script subroutine on the Maestro (substitute 0 with the desired subroutine number)
subroutine_number = 1
servo.runScriptSub(subroutine_number)

# Close the serial connection when done
servo.close()
# Create a serial connection
with serial.Serial(teensy_port, baudrate, timeout=1) as ser:
    while True:
        ser.write(b'<SH1>')  # Send the command
        print("Command sent: <SH1>")
        
        # Wait for 10 seconds
        time.sleep(10)