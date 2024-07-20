import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import numpy as np


class VectorPublisherNode(Node):
    def __init__(self):
        super().__init__('vector_publisher_node')

        # Initialize subscriber
        self.subscription = self.create_subscription(
            PoseStamped,
            '/converted_pose',
            self.pose_callback,
            10
        )

        # Initialize publisher
        self.vector_publisher = self.create_publisher(
            Float32MultiArray,
            'vector_topic',
            10
        )

        self.last_position = None
        self.current_position = None

    def pose_callback(self, msg):
        self.current_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

        if self.last_position is not None:
            vector = self.current_position - self.last_position
            self.publish_vector(vector)

        self.last_position = self.current_position

    def publish_vector(self, vector):
        vector_msg = Float32MultiArray()
        vector_msg.data = vector.tolist()
        self.vector_publisher.publish(vector_msg)
        self.get_logger().info(f'Published vector: {vector.tolist()}')


def main(args=None):
    rclpy.init(args=args)
    node = VectorPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
