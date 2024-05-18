import rclpy
from rclpy.node import Node
from drone_project_msgs.msg import CompassGPSHeading
from std_msgs.msg import Float64


class HeadingPublisher(Node):
    def __init__(self):
        super().__init__('heading_publisher')
        self.publisher = self.create_publisher(CompassGPSHeading, 'compass_gps_heading', 10)
        self.compass_hdg = None
        self.gps_hdg = None

    def compass_callback(self, msg):
        self.compass_hdg = msg.data
        self.publish_heading()

    def gps_callback(self, msg):
        self.gps_hdg = msg.data
        self.publish_heading()

    def publish_heading(self):
        if self.compass_hdg is not None and self.gps_hdg is not None:
            heading_msg = CompassGPSHeading()
            heading_msg.compass_hdg = self.compass_hdg
            heading_msg.gps_hdg = self.gps_hdg
            self.publisher.publish(heading_msg)


def main(args=None):
    rclpy.init(args=args)
    heading_publisher = HeadingPublisher()
    rclpy.spin(heading_publisher)
    heading_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
