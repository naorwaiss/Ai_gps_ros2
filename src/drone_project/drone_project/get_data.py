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
from mavros_msgs.msg import GPSRAW
from sensor_msgs.msg import Imu




class Listener(Node):
    def __init__(self):
        super().__init__('get_data')
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)

##############################################statuse##########################################
        self.topic_reception_status = {
            '/rpm_data': False,
            '/voltage_data': False,
            '/temperature_data': False,
            '/optical_flow_data': False,
            '/converted_pose': False,
            '/mavros/gpsstatus/gps1/raw': False,
            '/mavros/imu_data': False
        }




#######################################call for no gps functino #############################################



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
            self.Voltege_motor_callback,  # Assuming you have a callback function named rpm_callback
            qos_profile
        )


        # Subscription to '/temperature_data'
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/temperature_data',
            self.tmp_callback,  # Assuming you have a callback function named rpm_callback
            qos_profile
        )

        # Subscription to '/optical_flow_data'
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/optical_flow_data',
            self.optic_flow_collback,  # Assuming you have a callback function name doptic_flow_collback
            qos_profile
        )

        # Subscription to '/mavros/imu/data'
        self.subscription = self.create_subscription(
            Imu,  # Change Float32MultiArray to Imu
            '/mavros/imu/data',  # Correct topic name
            self.imu_data_coallback,  # Assuming you have a callback function named imu_data_coallback
            qos_profile
        )

        ###################################with GPS part##########################

        # Subscription to '/converter_pose' instead of '/mavros/local_position/pose'
        self.pose_subscription = self.create_subscription(
            Float32MultiArray,
            '/converted_pose',
            self.pose_callback,
            qos_profile)


        # Subscription to '/mavros/gpsstatus/gps1/raw' for GPS data
        self.subscription = self.create_subscription(
            GPSRAW,
            '/mavros/gpsstatus/gps1/raw',
            self.gps_callback,
            qos_profile
        )





########################################init###################################################3

        self.hdop = None
        self.x = None
        self.y = None
        self.z = None
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


###############################----function -------------------#############################



    def pose_callback(self, msg):
        if len(msg.data) == 6:
            #self.time = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9

            x, z, y, roll, pitch, yaw = msg.data
            self.x = x
            self.y = y
            self.z = z
            self.roll = roll
            self.pitch = pitch
            self.yaw = yaw
            #print(f"x: {self.x}, y: {self.y}, z: {self.z}, roll: {self.roll}, pitch: {self.pitch}, yaw: {self.yaw}")
            self.topic_reception_status['/converted_pose'] = True

        else:
            self.get_logger().warn("Invalid data received on /converted_pose topic.")


    def gps_callback(self, msg):
        hdop = msg.eph
        self.hdop = (float(hdop))/100
        self.topic_reception_status['/mavros/gpsstatus/gps1/raw'] = True



    def rpm_callback(self, msg):
        # Assuming the message contains data for four motors
        if len(msg.data) == 4:
            self.m1, self.m2, self.m3, self.m4 = msg.data
            # Now m1, m2, m3, and m4 contain the values from the rpm_data message
            self.topic_reception_status['/rpm_data'] = True

        else:
            self.get_logger().warn("Invalid data received on /rpm_data topic. Expected data for four motors.")

    def tmp_callback(self, msg):
        # Assuming the message contains data for four motors
        if len(msg.data) == 4:
            self.temp_m1, self.temp_m2, self.temp_m3, self.temp_m4 = msg.data
            self.topic_reception_status['/temperature_data'] = True
        else:
            self.get_logger().warn("Invalid data received on /temperature_data Expected data for four motors.")




    def Voltege_motor_callback(self, msg):
        # Assuming the message contains data for four motors
        if len(msg.data) == 4:
            self.volt_m1, self.volt_m2, self.volt_m3, self.volt_m4 = msg.data
            self.topic_reception_status['/voltage_data'] = True
        else:
            self.get_logger().warn("Invalid data received on /temperature_data Expected data for four motors.")

    def optic_flow_collback(self, msg):
        # Assuming the message contains data for four motors
        if len(msg.data) == 2:
            self.x_camera, self.y_camera = msg.data
            self.topic_reception_status['/optical_flow_data'] = True
        else:
            self.get_logger().warn("Invalid data received on //optical_flow_data.")

    def IMU_callback(self, msg):
        # Check if the angular_velocity field exists in the message
        if msg.angular_velocity:
            # Accessing angular velocity x, y, and z values
            self.angular_velocity_x = msg.angular_velocity.x
            self.angular_velocity_y = msg.angular_velocity.y
            self.angular_velocity_z = msg.angular_velocity.z
        else:
            self.get_logger().warn("Angular velocity data is missing in the IMU message.")

        # Check if the linear_acceleration field exists in the message
        if msg.linear_acceleration:
            # Accessing linear acceleration x, y, and z values
            self.linear_acceleration_x = msg.linear_acceleration.x
            self.linear_acceleration_y = msg.linear_acceleration.y
            self.linear_acceleration_z = msg.linear_acceleration.z

        else:
            # Handle the case where linear_acceleration field is missing
            self.get_logger().warn("Linear acceleration data is missing in the IMU message.")





class Data_csv:
    def __init__(self, listener):
        self.listener = listener
        self._running = False
        self.buffer_size = 50  # adjust the buffer size
        self.buffer = []

    def start(self):
        self._running = True
        self.thread = threading.Thread(target=self.process_data)
        self.thread.start()

    def stop(self):
        self._running = False
        self.thread.join()

    def start_scv_file(self):
        """
        Flush the buffer to CSV file
        """
        directory = "/home/naor/Ai_gps_ros2/src/drone_project/drone_project/log_flight"  # need to change it at my computer
        if not os.path.exists(directory):
            os.makedirs(directory)

        filename = os.path.join(directory, "flight_data.csv")

        with open(filename, mode='a', newline='') as file:
            csv_writer = csv.writer(file)
            # Write the header if the file is empty
            csv_writer.writerow([
                "x_gps", "y_gps", "z_gps"])  # Write the header

    def check_msg_arrive(self):
        while True:
            missing_topics = [topic for topic, received in self.topic_reception_status.items() if not received]
            if not missing_topics:
                print("All messages are received.")
                break
            else:
                print("Messages not received for topics:", missing_topics)
                # You can add further actions here if needed
            time.sleep(1)  # Add a short delay to avoid tight looping and excessive CPU usage

    def hdop_status(self):
        #ok this function work - check the status of the hdop before start the flight
        while True:
            if self.listener.hdop is not None and self.listener.hdop <= 1:
                break
            if self.listener.hdop is not None:
                print(f"HDOP: {self.listener.hdop}. Waiting for HDOP to be less than or equal to 1.5.")
            else:
                print("HDOP not received yet. Waiting...")
            time.sleep(1)

        print("HDOP is less than or equal to 1.5. Starting data processing and printing.")

    def process_data(self):
       # self.check_msg_arrive()  # check the msg that all arive- not work at the sitl..

        self.hdop_status() #check th hdop status

        self.start_scv_file() #open the csv file



        while self._running:
            for i in range(self.buffer_size):

                #print(f"x: {self.listener.x}, y: {self.listener.y}, z: {self.listener.z}")
                #print(f"m1 : {self.listener.m1}, m2: {self.listener.m2}, m3: {self.listener.m3 },m4: {self.listener.m4}")
                print(f"we ar at iteration {i}, the loop take  seconds.")



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
