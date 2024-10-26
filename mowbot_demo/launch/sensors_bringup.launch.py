#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    # Path to the YAML parameter file
    package_name = 'mowbot_demo' 
    param_dir = os.path.join(get_package_share_directory(package_name), 'param')
    param_file = os.path.join(param_dir, 'sensor_params.yaml')

    log_info = LogInfo(msg=f'Loading sensor parameters from: {param_file}')

    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[param_file],
        output='screen',
    )
    
    rsx_um7_node = Node(
        package='umx_driver',
        executable='um7_driver',
        name='um7_driver',
        parameters=[param_file],
        output='screen'
    )

    return LaunchDescription([
        log_info,   
        rplidar_node,
        rsx_um7_node
    ])
