import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, PointStamped, Point, TransformStamped

import tf2_geometry_msgs

import tf2_ros

import pytz
import pyproj
import numpy as np

def get_utm_coordinates(latitude, longitude):
    # Define the source (WGS84) and target (UTM) coordinate systems
    wgs84 = pyproj.CRS("EPSG:4326")  # WGS84 coordinate system
    crs = pyproj.CRS("EPSG:32652")  # UTM Zone 52N coordinate system (for Daegu South Korea)
    # Create a transformer for the conversion
    transformer = pyproj.Transformer.from_crs(wgs84, crs, always_xy=True)
    utm_easting, utm_northing = transformer.transform(longitude, latitude)
    return utm_easting, utm_northing

def get_wgs84_coordinates(utm_easting, utm_northing):
    # Define the source (UTM) and target (WGS84) coordinate systems
    wgs84 = pyproj.CRS("EPSG:4326")  # WGS84 coordinate system
    crs = pyproj.CRS("EPSG:32652")  # UTM Zone 52N coordinate system (for Daegu South Korea)
    # Create a transformer for the conversion
    transformer = pyproj.Transformer.from_crs(crs, wgs84, always_xy=True)
    longitude, latitude = transformer.transform(utm_easting, utm_northing)
    return latitude, longitude

class DualGpsCompass(Node):
    
    def __init__(self):
        super().__init__('dual_gps_compass')
        
        self.gps_left_frame_id = 'gps_left_link'
        self.gps_right_frame_id = 'gps_right_link'
        self.base_frame_id = 'base_link'
        self.combined_gps_frame_id = 'combined_gps_link'
        self.publish_combined_gps_tf = True
        
        self.gps_left_updated = False
        self.gps_right_updated = False
        
        self.last_gps_left = NavSatFix()
        self.last_gps_right = NavSatFix()
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)    
        
        self.gps_left_sub = self.create_subscription(NavSatFix, '/ublox_gpsl_node/fix', self.gps_left_callback, 10)
        self.gps_right_sub = self.create_subscription(NavSatFix, '/ublox_gpsr_node/fix', self.gps_right_callback, 10)
        
        self.gps_fix_pub = self.create_publisher(PoseStamped, '/fix', 10)
        self.dual_gps_heading_pub = self.create_publisher(PoseStamped, '/odometry/dual_gps_heading', 10)
        
        self.heading_process_timer = self.create_timer(1.0, self.heading_process_timer_callback)
        
    def gps_left_callback(self, msg: NavSatFix):
        
        if self.gps_left_updated:
            return
        
        # check if the GPS data is valid
        if msg.status.status == -1:
            self.get_logger().info('Invalid GPS data')
            return
        
        self.last_gps_left = msg
        self.gps_left_updated = True
        
        
    def gps_right_callback(self, msg: NavSatFix):
        
        if self.gps_right_updated:
            return
        
        # check if the GPS data is valid
        if msg.status.status == -1:
            self.get_logger().info('Invalid GPS data')
            return
        
        self.last_gps_right = msg
        self.gps_right_updated = True
        
        
    def heading_process_timer_callback(self):
        
        # Check if both GPS data are updated
        if not self.gps_left_updated and not self.gps_right_updated:
            self.get_logger().info('Waiting for GPS data')
            return
        
        self.get_logger().info('Start heading process')
        
        self.get_logger().info(f'last GPS Left: {self.last_gps_left.latitude}, {self.last_gps_left.longitude}')
        self.get_logger().info(f'last GPS Right: {self.last_gps_right.latitude}, {self.last_gps_right.longitude}')
        
        
        # Get the UTM coordinates
        utm_left_x, utm_left_y = get_utm_coordinates(self.last_gps_left.latitude, self.last_gps_left.longitude)
        utm_right_x, utm_right_y = get_utm_coordinates(self.last_gps_right.latitude, self.last_gps_right.longitude)
        
        self.get_logger().info(f'UTM Left: {utm_left_x}, {utm_left_y}')
        self.get_logger().info(f'UTM Right: {utm_right_x}, {utm_right_y}')
        
        # Calculate the relative position
        relative_x = utm_right_x - utm_left_x
        relative_y = utm_right_y - utm_left_y
        
        # Determine the heading angle
        heading_theta = np.arctan2(relative_y, relative_x) # in radian
        self.get_logger().info(f'Heading Angle: {heading_theta}')
        
        # get transform from gps_left_link and gps_right_link to base_link
        
        try:
            transform_left = self.tf_buffer.lookup_transform(self.base_frame_id, self.gps_left_frame_id, rclpy.time.Time())
            transform_right = self.tf_buffer.lookup_transform(self.base_frame_id, self.gps_right_frame_id, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'Error: {e}')
            return
        
        self.get_logger().info(f'Left Transform: {transform_left}')
        self.get_logger().info(f'Right Transform: {transform_right}')
        
        base_utm_left = tf2_geometry_msgs.do_transform_point(
            PointStamped(point=Point(x=utm_left_x, y=utm_left_y)), transform_left)
        base_utm_right = tf2_geometry_msgs.do_transform_point(
            PointStamped(point=Point(x=utm_right_x, y=utm_right_y)), transform_right)
        
        base_utm_left_x, base_utm_left_y = base_utm_left.point.x, base_utm_left.point.y
        base_utm_right_x, base_utm_right_y = base_utm_right.point.x, base_utm_right.point.y

        self.get_logger().info(f'Base UTM Left: {base_utm_left_x}, {base_utm_left_y}')
        self.get_logger().info(f'Base UTM Right: {base_utm_right_x}, {base_utm_right_y}')
           
        combined_base_utm_x = (base_utm_left_x + base_utm_right_x) / 2
        combined_base_utm_y = (base_utm_left_y + base_utm_right_y) / 2
        
        self.get_logger().info(f'Combined Base UTM: {combined_base_utm_x}, {combined_base_utm_y}')
        combined_base_lat, combined_base_lon = get_wgs84_coordinates(combined_base_utm_x, combined_base_utm_y)
        self.get_logger().info(f'Combined Base Lat Lon: {combined_base_lat}, {combined_base_lon}')
        
        # Publish the Combined GPS transform
        if self.publish_combined_gps_tf:
            base_to_combined_gps_tf = TransformStamped()
            base_to_combined_gps_tf.header.stamp = self.get_clock().now().to_msg()
            base_to_combined_gps_tf.header.frame_id = self.base_frame_id
            base_to_combined_gps_tf.child_frame_id = self.combined_gps_frame_id
            base_to_combined_gps_tf.transform.translation.x = 0.0
            base_to_combined_gps_tf.transform.translation.y = 0.0
            base_to_combined_gps_tf.transform.translation.z = 0.0
            base_to_combined_gps_tf.transform.rotation.x = 0.0
            base_to_combined_gps_tf.transform.rotation.y = 0.0
            base_to_combined_gps_tf.transform.rotation.z = 0.0
            base_to_combined_gps_tf.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(base_to_combined_gps_tf)
        
        
                
        # Reset the flag
        self.gps_left_updated = False
        self.gps_right_updated = False
        
        
    
    


def main(args=None):
    
    rclpy.init(args=args)
    dual_gps_compass = DualGpsCompass()
    rclpy.spin(dual_gps_compass)
    dual_gps_compass.destroy_node()
    rclpy.shutdown()
    