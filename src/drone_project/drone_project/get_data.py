import threading
import time
import csv
import os



#ros2 import
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from sensor_msgs.msg import MagneticField
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import NavSatFix



class Listener(Node):
    def __init__(self):
        super().__init__('get_data')
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)

        # Subscription to '/converter_pose' instead of '/mavros/local_position/pose'
        self.pose_subscription = self.create_subscription(
            Float32MultiArray,
            '/converted_pose',
            self.pose_callback,
            qos_profile)

        # Subscription to '/mavros/imu/mag'
        self.mag_subscription = self.create_subscription(
            MagneticField,
            '/mavros/imu/mag',
            self.mag_callback,
            qos_profile)

        # Subscription to '/mavros/global_position/raw/fix'
        self.subscription = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/raw/fix',
            self.gps_callback,
            qos_profile
        )

        # Subscription to '/rpm_data'
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/rpm_data',
            self.rpm_callback,  # Assuming you have a callback function named rpm_callback
            qos_profile
        )

        # Subscription to '/voltage_data'
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/voltage_data',
            self.rpm_callback,  # Assuming you have a callback function named rpm_callback
            qos_profile
        )


        # Subscription to '/temperature_data'
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/temperature_data',
            self.rpm_callback,  # Assuming you have a callback function named rpm_callback
            qos_profile
        )


        self.hdop = None
        self.x = None
        self.y = None
        self.z = None
        self.roll = None
        self.pitch = None
        self.yaw = None

    def pose_callback(self, msg):
        if len(msg.data) == 6:
            x, z, y, roll, pitch, yaw = msg.data
            self.x = x
            self.y = y
            self.z = z
            self.roll = roll
            self.pitch = pitch
            self.yaw = yaw
            #print(f"x: {self.x}, y: {self.y}, z: {self.z}, roll: {self.roll}, pitch: {self.pitch}, yaw: {self.yaw}")
        else:
            self.get_logger().warn("Invalid data received on /converter_pose topic.")

    def mag_callback(self, msg):
        pass

    def gps_callback(self, msg):
        self.hdop = msg.position_covariance[0]

    def rpm_callback(self, msg):
        # Assuming the message contains data for four motors
        if len(msg.data) == 4:
            self.m1, self.m2, self.m3, self.m4 = msg.data
            # Now m1, m2, m3, and m4 contain the values from the rpm_data message
        else:
            self.get_logger().warn("Invalid data received on /rpm_data topic. Expected data for four motors.")


class Data_csv:
    def __init__(self, listener):
        self.listener = listener
        self._running = False
        self.buffer = 50  # adjust the buffer size
        self.data = []

    def start(self):
        self._running = True
        self.thread = threading.Thread(target=self._process_data)
        self.thread.start()

    def stop(self):
        self._running = False
        self.thread.join()

    def start_scv_file(self):
        """
        Flush the buffer to CSV file
        """
        directory = "/home/naor/ros2_ws/src/drone_project/drone_project/log_flight"  # need to change it at my computer
        if not os.path.exists(directory):
            os.makedirs(directory)

        filename = os.path.join(directory, "flight_data.csv")

        with open(filename, mode='a', newline='') as file:
            csv_writer = csv.writer(file)
            # Write the header if the file is empty
            csv_writer.writerow([
                "x_gps", "y_gps", "z_gps"])  # Write the header



    def _process_data(self):
        while self.listener.hdop is None or self.listener.hdop > 1.5:
            print(f"HDOP: {self.listener.hdop}. Waiting for HDOP to be less than 1.")
            time.sleep(1)

        print("HDOP is less than 1. Starting data processing and printing.")

        self.start_scv_file()




        while self._running:
            print(f"x: {self.listener.x}, y: {self.listener.y}, z: {self.listener.z}")



def main(args=None):
    rclpy.init(args=args)
    listener = Listener()

    # Start the data processing and printing thread
    data_processor = Data_csv(listener)
    data_processor.start()

    try:
        rclpy.spin(listener)
    finally:
        data_processor.stop()
        listener.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
