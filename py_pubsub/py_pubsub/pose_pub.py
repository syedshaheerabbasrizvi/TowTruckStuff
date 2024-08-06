import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped # For publishing it in the format so that RViz2 can understand
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile # This is for setting Quality of Service (how data is exchanged between nodes).  All we're defining here is the depth (msg history), though I think we can probably live without it.
class PositionPublisher(Node): 

    def __init__(self):
        super().__init__('position_publisher') # Similar to before

        qos_profile = QoSProfile(depth=10) # changing the depth to 10 messages and storing this to be sent to other places

        # Subscribers for x, y, and theta data. Similar to before
        self.subscription_x = self.create_subscription(
            Float32,
            'x',
            self.x_callback,
            qos_profile)

        self.subscription_y = self.create_subscription(
            Float32,
            'y',
            self.y_callback,
            qos_profile)

        self.subscription_theta = self.create_subscription(
            Float32,
            'theta',
            self.theta_callback,
            qos_profile)

        # Publisher for the pose data
        self.pose_publisher = self.create_publisher(PoseStamped, 'pose', qos_profile)

        # Initialize variables to store the x, y, and theta values
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def x_callback(self, msg): # A callback is a function that is sent as an argument to another function! In this case, we are updating our value and pose. We are sending this to the subscription function so that it can automatically run this with each update in the subscription data
        self.x = msg.data
        self.publish_pose()

    def y_callback(self, msg):
        self.y = msg.data
        self.publish_pose()

    def theta_callback(self, msg):
        self.theta = msg.data
        self.publish_pose()

    def publish_pose(self):
        pose_msg = PoseStamped() # Create an object of the pose
        pose_msg.header.stamp = self.get_clock().now().to_msg() # I assume this is for adding timestamps using these functions... which I don't get completely
        pose_msg.header.frame_id = 'map'  # You can change this to a frame_id suitable for your setup

        pose_msg.pose.position.x = self.x # Assigning the x we have read to the x of the pose, and so on so forth
        pose_msg.pose.position.y = self.y
        pose_msg.pose.position.z = 0.0 # This is a 2D pose! No need for a third dimension.

        # Assuming theta is in radians
        import math # So the code up next converts 2D Euler angle (which is the yaw i.e. theta) to quaternions. In 2D motion, qz and qw are the equations shown, while qx and qy are 0.
        pose_msg.pose.orientation.z = math.sin(self.theta / 2.0) 
        pose_msg.pose.orientation.w = math.cos(self.theta / 2.0)

        self.pose_publisher.publish(pose_msg)
        self.get_logger().info(f'Published pose: x={self.x}, y=a{self.y}, theta={self.theta}')

def main(args=None): # Very similar to the subscriber main code
    rclpy.init(args=args)
    position_publisher = PositionPublisher()
    rclpy.spin(position_publisher)
    position_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
