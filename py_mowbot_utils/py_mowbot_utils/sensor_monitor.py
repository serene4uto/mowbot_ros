import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu, LaserScan, NavSatFix
from rtcm_msgs.msg import Message as Rtcm

class SensorMonitorNode(Node):
    def __init__(self):
        super().__init__('sensor_monitor_node')

        self.last_received_times = {
            '/imu/data': None,
            '/scan': None,
            '/rtcm': None,
            '/imu_gps_heading/data': None,
            '/ublox_gpsl_node/fix': None,
            '/ublox_gpsr_node/fix': None,
        }

        # Subscriptions for the topics
        self.create_subscription(Imu, '/imu/data', self.generic_callback('/imu/data'), 10)
        self.create_subscription(LaserScan, '/scan', self.generic_callback('/scan'), 10)
        self.create_subscription(Rtcm, '/rtcm', self.generic_callback('/rtcm'), 10)
        self.create_subscription(Imu, '/imu_gps_heading/data', self.generic_callback('/imu_gps_heading/data'), 10)
        self.create_subscription(NavSatFix, '/ublox_gpsl_node/fix', self.generic_callback('/ublox_gpsl_node/fix'), 10)
        self.create_subscription(NavSatFix, '/ublox_gpsr_node/fix', self.generic_callback('/ublox_gpsr_node/fix'), 10)

        # Publisher for sensor status
        self.status_publisher = self.create_publisher(String, '/sensor_status', 10)

        # Timer to check sensor status periodically
        self.create_timer(1.0, self.check_sensor_status)

    def generic_callback(self, topic):
        def callback(msg):
            self.last_received_times[topic] = self.get_clock().now()
        return callback

    def check_sensor_status(self):
        now = self.get_clock().now()
        status_messages = []

        for topic, last_time in self.last_received_times.items():
            if last_time is None:
                status = f"{topic}: No data yet"
                self.get_logger().info(status)
                status_messages.append(status)
            else:
                elapsed_time = (now - last_time).nanoseconds / 1e9
                if elapsed_time < 2.0:
                    status = f"{topic}: Active"
                    self.get_logger().info(status)
                    status_messages.append(status)
                else:
                    status = f"{topic}: Inactive"
                    self.get_logger().warn(status)
                    status_messages.append(status)

        # Publish the status messages as a single string
        self.status_publisher.publish(String(data="\n".join(status_messages)))

def main(args=None):
    rclpy.init(args=args)
    
    sensor_monitor_node = SensorMonitorNode()

    try:
        rclpy.spin(sensor_monitor_node)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_monitor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()