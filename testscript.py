import maestro
import time
import serial

# Define the serial ports and baud rate
serial_port_maestro = '/dev/ttyACM1'
serial_port_teensy = '/dev/ttyACM0'
baudrate = 9600

# Function to communicate with Maestro
def communicate_with_maestro():
    try:
        # Open a connection to the Maestro controller
        servo = maestro.Controller(serial_port_maestro)

        # Run the script subroutine on the Maestro
        subroutine_number = 1
        servo.runScriptSub(subroutine_number)

        # Close the serial connection
        servo.close()
    except Exception as e:
        print(f"Error with Maestro communication: {e}")

# Function to continuously communicate with Teensy
def continuously_communicate_with_teensy():
    try:
        # Open a connection to Teensy
        with serial.Serial(serial_port_teensy, baudrate, timeout=1) as ser:
            while True:
                ser.write(b'<SH1>')  # Send the command
                print("Command sent to Teensy: <SH1>")
                
                # Wait for 10 seconds before sending the next command
                time.sleep(10)
    except Exception as e:
        print(f"Error with Teensy communication: {e}")

# Communicate with Maestro
communicate_with_maestro()

# Delay to ensure the Maestro connection is closed before starting Teensy communication
time.sleep(1)

# Start continuous communication with Teensy
continuously_communicate_with_teensy()
