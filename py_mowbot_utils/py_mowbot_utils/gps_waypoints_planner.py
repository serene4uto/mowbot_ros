import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from py_mowbot_utils.gps_utils import latLonYaw2Geopose
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
from sensor_msgs.msg import NavSatFix
import os
import sys
import yaml


class GpsWaypointPlannerGUI(tk.Tk, Node):
    def __init__(self):
        tk.Tk.__init__(self)
        Node.__init__(self, 'gps_waypoints_planner')
        self.title("GPS Waypoints Planner GUI")
        
        self.resizable(False, False)

        self.instructions1_label = tk.Label(self, 
                text="!!! Click on the mapviz to select the waypoints",
                    justify=tk.CENTER)
        self.instructions2_label = tk.Label(self,
                text="!!! Double click on the table to edit the waypoints",
                    justify=tk.CENTER)

        self.gps_waypoints_label = tk.Label(self, text="Selected waypoints:")

        self.treeview = ttk.Treeview(self)
        self.treeview["columns"]=("Order","Latitude","Longitude","Yaw")

        # Format our columns
        self.treeview.column("#0", width=0, stretch=tk.NO)
        self.treeview.column("Order", anchor=tk.W, width=50)
        self.treeview.column("Latitude", anchor=tk.W, width=120)
        self.treeview.column("Longitude", anchor=tk.W, width=120)
        self.treeview.column("Yaw", anchor=tk.W, width=120)

        # Create Headings
        self.treeview.heading("#0", text="", anchor=tk.W)
        self.treeview.heading("Order", text="Order", anchor=tk.W)
        self.treeview.heading("Latitude", text="Latitude", anchor=tk.W)
        self.treeview.heading("Longitude", text="Longitude", anchor=tk.W)
        self.treeview.heading("Yaw", text="Yaw", anchor=tk.W)
        self.treeview.bind("<Double-1>", self.on_double_click)

        self.save_name_label = tk.Label(self, text="Map Name:")
        self.save_name_textbox = tk.Entry(self, width=45)
        self.save_name_textbox.insert(0, "gps_waypoints.yaml")

        self.save_button = tk.Button(self, text="Save",
                                           command=self.save_selected_waypoints)
        self.clear_button = tk.Button(self, text="Clear",
                                           command=self.clear_selected_waypoints)
        
        
        self.instructions1_label.grid(row=0, column=0, columnspan=3, sticky='ew')
        self.instructions2_label.grid(row=1, column=0, columnspan=3, sticky='ew')
        self.gps_waypoints_label.grid(row=2, column=0, columnspan=3, sticky='w')
        self.treeview.grid(row=3, column=0, columnspan=3, sticky='nsew')
        self.save_name_label.grid(row=4, column=0)
        self.save_name_textbox.grid(row=4, column=1)
        self.save_button.grid(row=5, column=0, sticky='ew')
        self.clear_button.grid(row=5, column=2, sticky='ew')


        self.mapviz_wp_sub = self.create_subscription(
            PointStamped, "/clicked_point", self.mapviz_wp_cb, 1)
        
        self.selected_wps_pub = self.create_publisher(
            NavSatFix, "/igw_gps_points", 1)

        self.last_gps_position = NavSatFix()
        self.last_heading = 0.0

        self.waypoints_counter = 0

    def mapviz_wp_cb(self, msg: PointStamped):
        if msg.header.frame_id != "wgs84":
            self.get_logger().warning(
                "Received point from mapviz that ist not in wgs84 frame. This is not a gps point and wont be followed")
            return
        
        self.get_logger().info(f"Received point from mapviz at lon: {msg.point.x}, lat: {msg.point.y}")
        
        self.last_gps_position.latitude = msg.point.y
        self.last_gps_position.longitude = msg.point.x
        self.selected_wps_pub.publish(self.last_gps_position)

        self.waypoints_counter += 1
        self.treeview.insert(parent='', index='end', text='',
                                values=(self.waypoints_counter, msg.point.y, msg.point.x, 0.0))
        
    def on_double_click(self, event):
        # Get the selected item
        item = self.treeview.selection()[0]

        # Determine the column clicked
        column = self.treeview.identify_column(event.x)
        # col_index = self.treeview["columns"].index(column)-1
        if column == '#1':
            col_index = 0
        elif column == '#2':
            col_index = 1
        elif column == '#3':
            col_index = 2
        elif column == '#4':
            col_index = 3
        else:
            return  # Clicked outside the defined columns

        # Create an entry widget to edit the cell value
        x, y, width, height = self.treeview.bbox(item, column)
        entry = tk.Entry(self, width=width)
        entry.place(x=x, y=y, width=width, height=height)

        def on_entry_confirm(event):
            # Update the treeview item
            self.treeview.set(item, column=column, value=entry.get())
            entry.destroy()

        def on_entry_click_outside(event):
            entry.destroy()

        # Bind events to handle editing completion
        entry.bind("<Return>", on_entry_confirm)
        entry.bind("<FocusOut>", on_entry_click_outside)

        # Set the entry with the current cell value and focus it
        entry.insert(0, self.treeview.item(item, 'values')[col_index])
        entry.select_range(0, tk.END)
        entry.focus_set()

    def save_selected_waypoints(self):

        waypoint_data_list = {"waypoints": []}
        logging_file_path = self.save_name_textbox.get()
            
        with open(logging_file_path, "w") as f:
            for item in self.treeview.get_children():
                row_data = self.treeview.item(item, 'values')
                data = {
                    "latitude": float(row_data[1]),
                    "longitude": float(row_data[2]),
                    "yaw": float(row_data[3]),
                }
                waypoint_data_list["waypoints"].append(data)
            yaml.dump(waypoint_data_list, f)
        messagebox.showinfo("Saved", f"Saved waypoints to {logging_file_path}")

    def clear_selected_waypoints(self):
        self.waypoints_counter = 0
        self.treeview.delete(*self.treeview.get_children())
        pass


def main(args=None):
    rclpy.init(args=args)

    igps_gui_logger = GpsWaypointPlannerGUI()

    while rclpy.ok():
        # we spin both the ROS system and the interface
        rclpy.spin_once(igps_gui_logger, timeout_sec=0.1)  # Run ros2 callbacks
        igps_gui_logger.update()  # Update the tkinter interface

    rclpy.shutdown()


if __name__ == '__main__':
    main()
        


