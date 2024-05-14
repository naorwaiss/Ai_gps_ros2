import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('pose_subscriber')
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        self.subscription = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_profile)

    def pose_callback(self, msg):
        #self.get_logger().info('Received pose: %s' % msg)
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    subscriber = PoseSubscriber()
    rclpy.spin(subscriber)

if __name__ == '__main__':
    main()
