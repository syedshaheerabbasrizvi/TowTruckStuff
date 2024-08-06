import rclpy
from std_msgs.msg import Float32
import serial
import time

def main():
    rclpy.init()
    node = rclpy.create_node('ros_arduino_communication')
    pub_float = node.create_publisher(Float32, 'frequency_data', 10)

    # Establish a serial connection to the Arduino
    try:
        arduino = serial.Serial('/dev/ttyACM0', 115200)
        node.get_logger().info('Connected to serial port')
    except serial.SerialException as e:
        node.get_logger().error(f'Failed to connect to serial port: {e}')
        return

    try:
        while rclpy.ok():
            if arduino.in_waiting > 0:
                line = arduino.readline().decode('utf-8').rstrip()
                try:
                    # Assuming the Arduino is only sending frequency data
                    frq = float(line)

                    # Publish the float value
                    msg_float = Float32()
                    msg_float.data = frq
                    pub_float.publish(msg_float)
                    node.get_logger().info(f'Published frequency data: {frq}')

                except ValueError:
                    node.get_logger().warn(f'Received invalid data: {line}')


    except KeyboardInterrupt:
        pass

    finally:
        if arduino.is_open:
            arduino.close()  # Ensure the serial connection is closed
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
