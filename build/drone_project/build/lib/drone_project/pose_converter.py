import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray


class PoseConverter(Node):
    def __init__(self):
        super().__init__('pose_converter')
        qos_profile = rclpy.qos.qos_profile_system_default
        self.subscription = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_profile)
        self.publisher = self.create_publisher(Float32MultiArray, '/converted_pose', qos_profile)

    def pose_callback(self, msg):
        try:
            # Extract position
            position = msg.pose.position
            x, y, z = position.x, position.y, position.z

            # Extract orientation
            orientation = msg.pose.orientation
            qx, qy, qz, qw = orientation.x, orientation.y, orientation.z, orientation.w

            # Convert quaternion to Euler angles
            roll, pitch, yaw = self.quaternion_to_euler(qx, qy, qz, qw)

            # Organize data into a dictionary
            pose_data = {
                'x': x,
                'y': y,
                'z': z,
                'roll': roll,
                'pitch': pitch,
                'yaw': yaw
            }

            # Publish the dictionary as Float32MultiArray message
            self.publish_pose_data(pose_data)

        except Exception as e:
            self.get_logger().error(f"Error processing pose: {e}")

    def publish_pose_data(self, data):
        # Create Float32MultiArray message for publishing
        pose_msg = Float32MultiArray()
        pose_msg.data = [data['x'], data['y'], data['z'], data['roll'], data['pitch'], data['yaw']]
        self.publisher.publish(pose_msg)

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to euler angles."""
        import math
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x ** 2 + y ** 2))
        pitch = math.asin(2 * (w * y - z * x))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y ** 2 + z ** 2))
        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    pose_converter = PoseConverter()
    rclpy.spin(pose_converter)
    pose_converter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
