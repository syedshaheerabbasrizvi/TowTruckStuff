import rclpy # ROS 2 client library for Python
from std_msgs.msg import Float32 # For publishing the frequency in this format
import serial # To use Serial (in Arduino IDE) for communication
import time # I don't think this is required any more after the changes I made, but I also don't want to risk anything by removing this

def main():
    rclpy.init() # Boot rclpy for use
    node = rclpy.create_node('ros_arduino_communication') # This creates the actual node
    pub_float = node.create_publisher(Float32, 'frequency_data', 10) # And this creates what we will be publishing

    # Establish a serial connection to the Arduino
    try:
        arduino = serial.Serial('/dev/ttyACM0', 115200) # This will depend on the Arduino port and the baud rate, so do change as required
        node.get_logger().info('Connected to serial port') # So that we know that things are working fine
    except serial.SerialException as e: # Whenever we get an exception, we store it to e...
        node.get_logger().error(f'Failed to connect to serial port: {e}') # ...and log it as an error
        return

    try:
        while rclpy.ok(): # While rclpy is working right
            if arduino.in_waiting > 0: # Arduino is available
                line = arduino.readline().decode('utf-8').rstrip() # We read the line, which is in bytes. Hence, we decode it to text, and strip away the whitespace to the right.
                try:
                    # Assuming the Arduino is only sending frequency data
                    frq = float(line) # Convert the number we got to float

                    # Publish the float value
                    msg_float = Float32()
                    msg_float.data = frq
                    pub_float.publish(msg_float) # The actual message being published
                    node.get_logger().info(f'Published frequency data: {frq}') # This will show on the terminal

                except ValueError:
                    node.get_logger().warn(f'Received invalid data: {line}') # Error handling, not too important


    except KeyboardInterrupt: # Ctrl-C to break out of the infinite loop
        pass # Do nothing so you can get out

    finally: # 
        if arduino.is_open:
            arduino.close() # Ensure the serial connection is closed
        node.destroy_node() # Get rid of the node we build
        rclpy.shutdown() # And shut down rclpy

if __name__ == '__main__':
    main() # Standard to run the file
