import rclpy # Again for ROS 2 usage
from rclpy.node import Node # For inheritance
from std_msgs.msg import Float32, Float64 # Float 64 was used back when I was giving pulses to this, but I then removed it due to lack of need. When I clean up the code, I'll deal with these little nuisances
import math # For trigonometry in general

class KinematicCalculator(Node): # This class is inheriting from Node, which is from ROS 2

    def __init__(self):
        super().__init__('kinematic_calculator') # Name of our node to run the constructor as ROS 2 requires

        # Subscriber for frequency data
        self.frequency_subscription = self.create_subscription( # This is how you subscribe to a topic
            Float32, # The data type of the published data
            'frequency_data', # The name
            self.frequency_callback, # Where we will store?
            10) # This is the buffer size of our QoS profile: 10 messages, so that slow subscribers won't be stuck with an ever-increasing lag.

        # Publishers for the calculated values, fairly self explanatory
        self.pub_x = self.create_publisher(Float32, 'x', 10)
        self.pub_y = self.create_publisher(Float32, 'y', 10)
        self.pub_theta = self.create_publisher(Float32, 'theta', 10)

        # Initialize variables for cumulative values
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Initial heading angle (radians)
        
        # Initialize variables for received data
        self.frequency = 0.0
        self.phi = 0.2  # Assuming a constant steering angle (radians)

    def frequency_callback(self, msg):
        self.frequency = msg.data
        self.calculate_and_publish()

    def calculate_and_publish(self):
        # Constants
        wheel_radius = 0.2032  # Radius of the wheel (meters)
        wheelbase = 1.17  # Distance between front and back wheels (meters)
        pulses_per_revolution = 740
        sampling_time = 0.01  # Sampling time (seconds)

        # Calculate the wheel speed (v). Our frequency divided by the pulses per revolution gives revolutions per second. We know that the length of one revolution is equal to the circumference of the wheel, so multiply this value by 2 pi r to get distance per time... which is v.
        v = (self.frequency * 2 * math.pi * wheel_radius) / pulses_per_revolution

        # Update the heading angle (theta)
        self.theta += (v / wheelbase) * math.tan(self.phi) * sampling_time # formula of theta dot is v/L * tan phi. Instead of defining a new variable, we directly calculate this value, multiply it by dt (the sampling time of 0.01 seconds), and add it to the previous value to perform summation since we cannot integrate this. The x and y calculations are done similarly with their own respective equations.

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
    rclpy.init(args=args) # Initialize. Apparently, this entire args business is to allow flexibility, which is that you can customize the arguments sent while calling this script in the command line. Unnecessary here, but may be useful in future ROS 2 work.
    kinematic_calculator = KinematicCalculator() # Create our object
    rclpy.spin(kinematic_calculator) # Spinning our node i.e. executing work
    kinematic_calculator.destroy_node() # After we're done, destroy the node and shut down
    rclpy.shutdown()

if __name__ == '__main__':
    main()
