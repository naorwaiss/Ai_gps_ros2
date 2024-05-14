import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ESCTelemetry
from std_msgs.msg import Float32MultiArray

class ESCDataProcessor(Node):
    def __init__(self):
        super().__init__('esc_data_processor')
        # Subscribe to the MAVROS ESC telemetry topic
        self.esc_telemetry_subscription = self.create_subscription(
            ESCTelemetry,
            '/mavros/esc_telemetry/telemetry',
            self.esc_callback,
            100)

        # Publishers for organized data
        self.rpm_publisher = self.create_publisher(Float32MultiArray, 'rpm_data', 100)
        self.voltage_publisher = self.create_publisher(Float32MultiArray, 'voltage_data', 100)
        self.temperature_publisher = self.create_publisher(Float32MultiArray, 'temperature_data', 100)

    def esc_callback(self, msg):
        # Initialize lists to hold the organized data for each motor
        temperatures = []
        voltages = []
        rpms = []

        # Loop through each ESCTelemetryItem in the esc_telemetry array
        for esc_item in msg.esc_telemetry:
            temperatures.append(esc_item.temperature)
            voltages.append(esc_item.voltage)
            rpms.append(esc_item.rpm)

        # Publish the organized data
        # Publish RPM data
        rpm_data = Float32MultiArray()
        rpm_data.data = rpms
        self.rpm_publisher.publish(rpm_data)
        
        # Publish Voltage data
        voltage_data = Float32MultiArray()
        voltage_data.data = voltages
        self.voltage_publisher.publish(voltage_data)
        
        # Publish Temperature data
        temperature_data = Float32MultiArray()
        temperature_data.data = temperatures
        self.temperature_publisher.publish(temperature_data)

        # Log the data publishing action
        #self.get_logger().info('Published organized ESC telemetry data')

def main(args=None):
    rclpy.init(args=args)
    esc_data_processor = ESCDataProcessor()
    rclpy.spin(esc_data_processor)
    esc_data_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
