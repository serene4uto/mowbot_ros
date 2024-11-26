import threading
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
import yaml
import tkinter as tk
from tkinter import filedialog
from py_mowbot_utils.gps_utils import latLonYaw2Geopose
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from robot_localization.srv import FromLL


class YamlWaypointParser:
    """
    Parse a set of GPS waypoints from a YAML file.
    """

    def __init__(self, wps_file_path: str) -> None:
        with open(wps_file_path, 'r') as wps_file:
            self.wps_dict = yaml.safe_load(wps_file)

    def get_wps(self):
        """
        Get an array of geographic_msgs.msg.GeoPose objects from the YAML file.
        """
        gepose_wps = []
        for wp in self.wps_dict["waypoints"]:
            latitude, longitude, yaw = wp["latitude"], wp["longitude"], wp["yaw"]
            gepose_wps.append(latLonYaw2Geopose(latitude, longitude, yaw))
        return gepose_wps


class GpsWaypointFollowerGUI(tk.Tk):
    def __init__(self):
        super().__init__()

        # GUI Initialization
        self.title("GPS Waypoint Follower GUI")
        self.resizable(False, False)

        self.load_label = tk.Label(self, text="File path:")
        self.load_textbox = tk.Entry(self, width=45)
        self.load_textbox.insert(0, "gps_waypoints.yaml")
        self.select_path_button = tk.Button(
            self, text="Select", command=self.select_path)

        self.start_button = tk.Button(
            self, text="Start", command=self.start_wpf)
        self.stop_button = tk.Button(
            self, text="Stop", command=self.stop_wpf)
        self.stop_button.config(state=tk.DISABLED)

        self.load_label.grid(row=0, column=0, sticky='ew')
        self.load_textbox.grid(row=0, column=1, sticky='ew')
        self.select_path_button.grid(row=0, column=2, sticky='ew')
        self.start_button.grid(row=1, column=0, sticky='ew')
        self.stop_button.grid(row=1, column=2, sticky='ew')

        # ROS Initialization
        self.node = rclpy.create_node('gps_waypoint_follower')
        self.navigator = BasicNavigator()
        self.wp_parser = None

        self.localizer = self.node.create_client(FromLL, '/fromLL')
        while not self.localizer.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                'Service not available, waiting again...')

        self.state_check_timer = self.node.create_timer(
            1, self.check_nav_state)
        self.state_check_timer.cancel()

        self.selected_wps_pub = self.node.create_publisher(
            NavSatFix, "/igw_gps_points", 1)

        # Start ROS spinning in a separate thread
        self.ros_thread = threading.Thread(
            target=rclpy.spin, args=(self.node,), daemon=True)
        self.ros_thread.start()

        # Flag to indicate task completion
        self.task_complete = False

        # Periodically check for updates in the main GUI thread
        self.after(100, self.periodic_update)

    def select_path(self):
        self.load_textbox.delete(0, tk.END)
        self.load_textbox.insert(0, filedialog.askopenfilename())

    def start_wpf(self):
        self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)

        self.wp_parser = YamlWaypointParser(self.load_textbox.get())
        wps = self.wp_parser.get_wps()

        # Publish waypoints without blocking the main thread
        for wp in wps:
            loaded_wp_msg = NavSatFix()
            loaded_wp_msg.latitude = wp.position.latitude
            loaded_wp_msg.longitude = wp.position.longitude
            self.selected_wps_pub.publish(loaded_wp_msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.navigator.waitUntilNav2Active(localizer='controller_server')

        wpl = []
        for wp in wps:
            req = FromLL.Request()
            req.ll_point.longitude = wp.position.longitude
            req.ll_point.latitude = wp.position.latitude
            req.ll_point.altitude = wp.position.altitude

            log = 'long={:f}, lat={:f}, alt={:f}'.format(
                req.ll_point.longitude, req.ll_point.latitude, req.ll_point.altitude)
            self.node.get_logger().info(log)

            future = self.localizer.call_async(req)
            rclpy.spin_until_future_complete(self.node, future)

            resp = PoseStamped()
            resp.header.frame_id = 'map'
            resp.header.stamp = self.node.get_clock().now().to_msg()
            resp.pose.position.x = future.result().map_point.x
            resp.pose.position.y = future.result().map_point.y
            resp.pose.position.z = future.result().map_point.z

            log = 'x={:f}, y={:f}, z={:f}'.format(
                future.result().map_point.x, future.result().map_point.y, future.result().map_point.z)
            self.node.get_logger().info(log)

            resp.pose.orientation = wp.orientation
            wpl.append(resp)

        self.navigator.followWaypoints(wpl)

        # Start the state check timer
        if self.state_check_timer.is_canceled():
            self.state_check_timer.reset()

    def stop_wpf(self):
        self.navigator.cancelTask()
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)

        # Stop the state check timer
        if not self.state_check_timer.is_canceled():
            self.state_check_timer.cancel()

    def check_nav_state(self):
        if self.navigator.isTaskComplete():
            self.node.get_logger().info("Waypoints completed successfully")
            # Set the flag to update GUI in the main thread
            self.task_complete = True

    def periodic_update(self):
        # This method is called periodically from the GUI main thread
        if self.task_complete:
            # Task is complete, update GUI
            self.stop_wpf()
            self.task_complete = False  # Reset flag
        # Schedule next call
        self.after(100, self.periodic_update)


def main(args=None):
    rclpy.init(args=args)

    gps_waypoint_flw = GpsWaypointFollowerGUI()
    gps_waypoint_flw.mainloop()

    # Clean up ROS resources after GUI closes
    gps_waypoint_flw.node.destroy_node()
    rclpy.shutdown()
