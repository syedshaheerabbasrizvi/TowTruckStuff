import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64
import math

class KinematicCalculator(Node):

    def __init__(self):
        super().__init__('kinematic_calculator')

        # Subscriber for frequency data
        self.frequency_subscription = self.create_subscription(
            Float32,
            'frequency_data',
            self.frequency_callback,
            10)

        # Publishers for the calculated values
        self.pub_x = self.create_publisher(Float32, 'x', 10)
        self.pub_y = self.create_publisher(Float32, 'y', 10)
        self.pub_theta = self.create_publisher(Float32, 'theta', 10)

        # Initialize variables for cumulative values
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Initial heading angle (radians)
        
        # Initialize variables for received data
        self.frequency = 0.0
        self.phi = 1  # Assuming a constant steering angle (radians)

    def frequency_callback(self, msg):
        self.frequency = msg.data
        self.calculate_and_publish()

    def calculate_and_publish(self):
        # Constants
        wheel_radius = 0.2032  # Radius of the wheel (meters)
        wheelbase = 1.17  # Distance between front and back wheels (meters)
        pulses_per_revolution = 740
        sampling_time = 0.1  # Sampling time (seconds)

        # Calculate the wheel speed (v)
        v = (self.frequency * 2 * math.pi * wheel_radius) / pulses_per_revolution

        # Update the heading angle (theta)
        self.theta += (v / wheelbase) * math.tan(self.phi) * sampling_time

        # Update x, y positions using integration over time
        self.x += v * math.cos(self.theta) * sampling_time
        self.y += v * math.sin(self.theta) * sampling_time
	
        # Publish the cumulative values
        x_msg = Float32()
        x_msg.data = self.x
        self.pub_x.publish(x_msg)
        self.get_logger().info(f'Published x: {self.x}')

        y_msg = Float32()
        y_msg.data = self.y
        self.pub_y.publish(y_msg)
        self.get_logger().info(f'Published y: {self.y}')

        theta_msg = Float32()
        theta_msg.data = self.theta
        self.pub_theta.publish(theta_msg)
        self.get_logger().info(f'Published theta: {self.theta}')


def main(args=None):
    rclpy.init(args=args)
    kinematic_calculator = KinematicCalculator()
    rclpy.spin(kinematic_calculator)
    kinematic_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

