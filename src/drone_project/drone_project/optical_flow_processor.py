import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np

class OpticalFlowProcessor(Node):
    def __init__(self):
        super().__init__('optical_flow_processor')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            10
        )
        self.publisher_ = self.create_publisher(Float32MultiArray, 'optical_flow_data', 10)
        self.br = CvBridge()
        self.prev_frame = None

        # Enhancing the Lucas-Kanade parameters
        self.lk_params = dict(winSize=(21, 21),
                              maxLevel=2,
                              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

        # Smoothing factor for exponential smoothing
        self.alpha = 0.3
        self.prev_x = 0
        self.prev_y = 0

    def listener_callback(self, msg):
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_blur = cv2.GaussianBlur(gray, (5, 5), 0)

        if self.prev_frame is not None:
            # More sensitive feature detection
            prev_pts = cv2.goodFeaturesToTrack(self.prev_frame, maxCorners=150, qualityLevel=0.03, minDistance=7)
            if prev_pts is not None:
                # Calculate optical flow with enhanced parameters
                next_pts, status, _ = cv2.calcOpticalFlowPyrLK(self.prev_frame, gray_blur, prev_pts, None, **self.lk_params)
                if next_pts is not None and status is not None:
                    good_new = next_pts[status == 1]
                    good_old = prev_pts[status == 1]

                    if len(good_new) > 0 and len(good_old) > 0:
                        movement = np.mean(good_new - good_old, axis=0)
                        x_movement, y_movement = movement

                        # Exponential smoothing applied to flow vectors
                        x_movement = self.alpha * x_movement + (1 - self.alpha) * self.prev_x
                        y_movement = self.alpha * y_movement + (1 - self.alpha) * self.prev_y

                        self.prev_x = x_movement
                        self.prev_y = y_movement

                        y_movement_new = x_movement

                        x_movement_new = y_movement*-1

                        # Publishing the smoothed flow data
                        flow_msg = Float32MultiArray()
                        flow_msg.data = [x_movement_new, y_movement_new]
                        self.publisher_.publish(flow_msg)

                        # Uncomment below line if you want to print when running the node
                        # self.get_logger().info(f'Publishing Optical Flow Data: {flow_msg.data}')

        self.prev_frame = gray_blur

def main(args=None):
    rclpy.init(args=args)
    processor = OpticalFlowProcessor()
    rclpy.spin(processor)
    processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
