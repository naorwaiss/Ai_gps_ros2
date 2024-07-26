import threading
import time
import csv
import os
from collections import deque

# ROS2 import
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float32MultiArray, Float64
from sensor_msgs.msg import MagneticField, NavSatFix, Imu , BatteryState
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import GPSRAW, Altitude, RCIn

class Listener(Node):
    def __init__(self):
        super().__init__('get_data')
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        ##############################################status##########################################
        self.topic_reception_status = {
            '/rpm_data': False,
            '/voltage_data': False,
            '/temperature_data': False,
            '/optical_flow_data': False,
            '/converted_pose': False,
            '/mavros/gpsstatus/gps1/raw': False,
            '/mavros/imu/data': False,
            '/mavros/global_position/compass_hdg': False,
            '/mavros/altitude': False,
            '/mavros/rc/in': False
        }

        #######################################call for no gps function #############################################

        # Subscription to '/rpm_data'
        self.create_subscription(
            Float32MultiArray,
            '/rpm_data',
            self.rpm_callback,
            qos_profile
        )

        # Subscription to '/voltage_data'
        self.create_subscription(
            Float32MultiArray,
            '/voltage_data',
            self.voltage_motor_callback,
            qos_profile
        )

        # Subscription to '/temperature_data'
        self.create_subscription(
            Float32MultiArray,
            '/temperature_data',
            self.tmp_callback,
            qos_profile
        )

        # Subscription to '/optical_flow_data'
        self.create_subscription(
            Float32MultiArray,
            '/optical_flow_data',
            self.optic_flow_callback,
            qos_profile
        )

        # Subscription to '/mavros/imu/data'
        self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_data_callback,
            qos_profile
        )

        # Subscription to '/mavros/altitude' for altitude data
        self.create_subscription(
            Altitude,
            '/mavros/altitude',
            self.altitude_callback,
            qos_profile
        )

        # Subscription to '/mavros/battery'
        self.create_subscription(
            BatteryState,
            '/mavros/battery',
            self.battery_callback,
            qos_profile
        )

        ###################################with GPS part##########################

        # Subscription to '/converted_pose'
        self.create_subscription(
            Float32MultiArray,
            '/converted_pose',
            self.pose_callback,
            qos_profile
        )

        # Subscription to '/mavros/gpsstatus/gps1/raw' for GPS data
        self.create_subscription(
            GPSRAW,
            '/mavros/gpsstatus/gps1/raw',
            self.gps_callback,
            qos_profile
        )

        # Subscription to '/mavros/global_position/compass_hdg' for compass heading data
        self.create_subscription(
            Float64,
            '/mavros/global_position/compass_hdg',
            self.compass_callback,
            qos_profile
        )

        # Subscription to '/mavros/local_position/velocity_body'
        self.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_body',
            self.velocity_body_callback,
            qos_profile
        )

        # Subscription to '/mavros/rc/in' for RC input data
        self.create_subscription(
            RCIn,
            '/mavros/rc/in',
            self.rc_in_callback,
            qos_profile
        )

        ########################################init###################################################

        self.hdop = None
        self.x_gps = None
        self.y_gps = None
        self.z_gps = None
        self.roll = None
        self.pitch = None
        self.yaw = None
        self.x_camera = None
        self.y_camera = None
        self.buff = 30
        self.time = None
        self.linear_acceleration_x = None
        self.linear_acceleration_y = None
        self.linear_acceleration_z = None
        self.angular_velocity_x = None
        self.angular_velocity_y = None
        self.angular_velocity_z = None
        self.compass = None
        self.drone_stat = None
        self.mode = None
        self.m1 = None
        self.m2 = None
        self.m3 = None
        self.m4 = None
        self.temp_m1 = None
        self.temp_m2 = None
        self.temp_m3 = None
        self.temp_m4 = None
        self.volt_m1 = None
        self.volt_m2 = None
        self.volt_m3 = None
        self.volt_m4 = None
        self.non_gps_altitude = None
        self.rc_channel_7_value = None  # Initialize this attribute
        self.cell_voltage = None

        ###############################----function -------------------#############################

    def pose_callback(self, msg):
        if len(msg.data) == 6:
            x, y, z, roll, pitch, yaw = msg.data
            self.x_gps = x
            self.y_gps = y
            self.z_gps = z
            self.roll = roll
            self.pitch = pitch
            self.yaw = yaw
            self.topic_reception_status['/converted_pose'] = True
        else:
            self.get_logger().warn("Invalid data received on /converted_pose topic.")

    def gps_callback(self, msg):
        hdop = msg.eph
        self.hdop = (float(hdop)) / 100
        self.topic_reception_status['/mavros/gpsstatus/gps1/raw'] = True

    def rpm_callback(self, msg):
        if len(msg.data) == 4:
            self.m1, self.m2, self.m3, self.m4 = msg.data
            self.topic_reception_status['/rpm_data'] = True
        else:
            self.get_logger().warn("Invalid data received on /rpm_data topic. Expected data for four motors.")

    def tmp_callback(self, msg):
        if len(msg.data) == 4:
            self.temp_m1, self.temp_m2, self.temp_m3, self.temp_m4 = msg.data
            self.topic_reception_status['/temperature_data'] = True
        else:
            self.get_logger().warn("Invalid data received on /temperature_data topic. Expected data for four motors.")

    def voltage_motor_callback(self, msg):
        if len(msg.data) == 4:
            self.volt_m1, self.volt_m2, self.volt_m3, self.volt_m4 = msg.data
            self.topic_reception_status['/voltage_data'] = True
        else:
            self.get_logger().warn("Invalid data received on /voltage_data topic. Expected data for four motors.")

    def optic_flow_callback(self, msg):
        if len(msg.data) == 2:
            self.x_camera, self.y_camera = msg.data
            self.topic_reception_status['/optical_flow_data'] = True
        else:
            self.get_logger().warn("Invalid data received on /optical_flow_data topic.")

    def battery_callback(self, msg):
        # Extract the cell voltage data
        self.cell_voltage = msg.cell_voltage


    def imu_data_callback(self, msg):
        if msg.angular_velocity:
            self.angular_velocity_x = msg.angular_velocity.x
            self.angular_velocity_y = msg.angular_velocity.y
            self.angular_velocity_z = msg.angular_velocity.z
        else:
            self.get_logger().warn("Angular velocity data is missing in the IMU message.")

        if msg.linear_acceleration:
            self.linear_acceleration_x = msg.linear_acceleration.x
            self.linear_acceleration_y = msg.linear_acceleration.y
            self.linear_acceleration_z = msg.linear_acceleration.z
            self.topic_reception_status['/mavros/imu/data'] = True
        else:
            self.get_logger().warn("Linear acceleration data is missing in the IMU message.")

    def compass_callback(self, msg):
        self.compass = msg.data
        self.topic_reception_status['/mavros/global_position/compass_hdg'] = True

    def velocity_body_callback(self, msg):
        if msg is not None:
            self.x_velocity = msg.twist.linear.x
            self.y_velocity = msg.twist.linear.y
            self.z_velocity = msg.twist.linear.z
            self.topic_reception_status['/mavros/local_position/velocity_body'] = True
        else:
            self.get_logger().warn("Invalid data received on /mavros/local_position/velocity_body.")

    def altitude_callback(self, msg):
        if msg is not None:
            self.non_gps_altitude = msg.local
            self.topic_reception_status['/mavros/altitude'] = True
        else:
            self.get_logger().warn("Invalid data received on /mavros/altitude.")

    def rc_in_callback(self, msg):
        # Assuming channels are 0-indexed
        if len(msg.channels) > 6:
            self.rc_channel_7_value = msg.channels[6]
            self.topic_reception_status['/mavros/rc/in'] = True

class Data_csv:
    def __init__(self, listener):
        self.listener = listener
        self._running = False
        self.buffer_size = 50
        self.buffer = []

    def start(self):
        self._running = True
        self.thread = threading.Thread(target=self.process_data)
        self.thread.start()

    def stop(self):
        self._running = False
        self.thread.join()

    def start_csv_file(self):
        """
        Create the CSV file and write the header.
        """
        self.directory = "/home/drone/Ai_gps_ros2/src/drone_project/drone_project/log_flight"  # Update to your actual directory
        if not os.path.exists(self.directory):
            os.makedirs(self.directory)

        self.filename = os.path.join(self.directory, "flight_data.csv")

        with open(self.filename, mode='w', newline='') as file:
            csv_writer = csv.writer(file)
            # Write the header
            csv_writer.writerow([
                "m1_rpm", "m2_rpm", "m3_rpm", "m4_rpm",
                "temp_m1", "temp_m2", "temp_m3", "temp_m4",
                "volt_m1", "volt_m2", "volt_m3", "volt_m4",
                "x_camera", "y_camera",
                "Altitude",
                "linear_acceleration_x", "linear_acceleration_y", "linear_acceleration_z",  # imu
                "angular_velocity_x", "angular_velocity_y", "angular_velocity_z",  # imu
                "compass", "battery_voltage",
                "roll", "pitch", "yaw",
                "x_gps", "y_gps", "z_gps"
            ])

    def flush_buffer_to_csv(self):
        """
        Flush the buffer to CSV file.
        """
        with open(self.filename, mode='a', newline='') as file:
            csv_writer = csv.writer(file)
            for data in self.buffer:
                csv_writer.writerow(data)
        self.buffer.clear()

    def check_msg_arrive(self):
        while True:
            missing_topics = [topic for topic, received in self.listener.topic_reception_status.items() if not received]
            if not missing_topics:
                print("All messages are received.")
                break
            else:
                print("Messages not received for topics:", missing_topics)
            time.sleep(1)

    def hdop_status(self):
        while True:
            if self.listener.hdop is not None and self.listener.hdop <= 3:
                #need to check this value
                break
            if self.listener.hdop is not None:
                print(f"HDOP: {self.listener.hdop}. Waiting for HDOP to be less than or equal to 10.")
            else:
                print("HDOP not received yet. Waiting...")
            time.sleep(1)

        print("HDOP is less than or equal to 10. Starting data processing and printing.")

    def rc_channel_7_status(self):
        while True:
            if self.listener.rc_channel_7_value is not None and self.listener.rc_channel_7_value > 1500:
                print("RC channel 7 is greater than 1500. Starting data processing.")
                break
            else:
                print("Waiting for RC channel 7 to be greater than 1500...")
            time.sleep(1)

    def process_data(self):
        self.hdop_status()  # Check the HDOP status
        self.start_csv_file()  # Open the CSV file

        while True:  # Outer loop to keep checking RC channel 7
            self.rc_channel_7_status()  # Wait until RC channel 7 is above 1500 to start

            self._running = True
            while self._running:
                for i in range(self.buffer_size):
                    # Check if the RC channel 7 value drops below or equals 1500
                    if self.listener.rc_channel_7_value is not None and self.listener.rc_channel_7_value <= 1500:
                        print("RC channel 7 is less than or equal to 1500. Stopping data processing.")
                        self._running = False
                        break

                    data = [
                        self.listener.m1, self.listener.m2, self.listener.m3, self.listener.m4,
                        self.listener.temp_m1, self.listener.temp_m2, self.listener.temp_m3, self.listener.temp_m4,
                        self.listener.volt_m1, self.listener.volt_m2, self.listener.volt_m3, self.listener.volt_m4,
                        self.listener.x_camera, self.listener.y_camera,
                        self.listener.non_gps_altitude,
                        self.listener.linear_acceleration_x, self.listener.linear_acceleration_y,
                        self.listener.linear_acceleration_z,
                        self.listener.angular_velocity_x, self.listener.angular_velocity_y,
                        self.listener.angular_velocity_z,
                        self.listener.compass, self.listener.cell_voltage,
                        self.listener.roll, self.listener.pitch, self.listener.yaw,
                        self.listener.x_gps, self.listener.y_gps, self.listener.z_gps
                    ]
                    hz = 20
                    time.sleep(1 / hz)
                    self.buffer.append(data)

                    print(f"We are at iteration {i}")

                self.flush_buffer_to_csv()  # After getting buffer size data, upload the data to the CSV
                print("Buffer flushed to CSV.")

            print("Waiting for RC channel 7 to be greater than 1500 to restart data processing.")


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
