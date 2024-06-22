import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class FCOpticalInherit(Node):
    def __init__(self):
        super().__init__('fc_optical_inherit')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'optical_flow_data',
            self.listener_callback,
            10
        )
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('FC Optical Inherit Node has been started.')

    def listener_callback(self, msg):
        # Extract optical flow data
        x_movement = msg.data[0]
        y_movement = msg.data[1]

        # Create and populate Twist message
        twist = Twist()
        twist.linear.x = x_movement  # Forward/backward
        twist.linear.y = y_movement  # Left/right
        twist.linear.z = 0.0         # Up/down (no movement in this example)
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0        # Yaw (no rotation in this example)

        # Publish the Twist message to control the drone
        self.publisher_.publish(twist)

        # Uncomment below line if you want to print when running the node
        # self.get_logger().info(f'Publishing Velocity Command: {twist}')

def main(args=None):
    rclpy.init(args=args)
    controller = FCOpticalInherit()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
