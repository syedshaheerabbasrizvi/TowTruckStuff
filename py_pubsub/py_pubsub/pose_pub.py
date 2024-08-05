import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile

class PositionPublisher(Node):

    def __init__(self):
        super().__init__('position_publisher')

        qos_profile = QoSProfile(depth=10)

        # Subscribers for x, y, and theta data
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

    def x_callback(self, msg):
        self.x = msg.data
        self.publish_pose()

    def y_callback(self, msg):
        self.y = msg.data
        self.publish_pose()

    def theta_callback(self, msg):
        self.theta = msg.data
        self.publish_pose()

    def publish_pose(self):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'  # You can change this to a frame_id suitable for your setup

        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.position.z = 0.0

        # Assuming theta is in radians
        import math
        pose_msg.pose.orientation.z = math.sin(self.theta / 2.0)
        pose_msg.pose.orientation.w = math.cos(self.theta / 2.0)

        self.pose_publisher.publish(pose_msg)
        self.get_logger().info(f'Published pose: x={self.x}, y={self.y}, theta={self.theta}')

def main(args=None):
    rclpy.init(args=args)
    position_publisher = PositionPublisher()
    rclpy.spin(position_publisher)
    position_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

