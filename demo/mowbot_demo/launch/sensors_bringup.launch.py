#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Path to the YAML parameter file
    package_name = 'mowbot_demo' 
    param_dir = os.path.join(get_package_share_directory(package_name), 'param')
    rviz_dir = os.path.join(get_package_share_directory(package_name), 'rviz')
    param_file = os.path.join(param_dir, 'sensor_params.yaml')
    rviz_file = os.path.join(rviz_dir, 'sensors.rviz')

    declare_use_lidar_arg = DeclareLaunchArgument(
        'use_lidar',
        default_value='false',
        description='Whether to use the lidar or not'
    )
    declare_use_imu_arg = DeclareLaunchArgument(
        'use_imu',
        default_value='false',
        description='Whether to use the IMU or not'
    )

    declare_use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to start RViz or not'
    )

    use_lidar = LaunchConfiguration('use_lidar')
    use_imu = LaunchConfiguration('use_imu')
    use_rviz = LaunchConfiguration('use_rviz')

    log_info = LogInfo(msg=f'Loading sensor parameters from: {param_file}')

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('mowbot_description'), 'launch', 'description.launch.py']
        ))
    )

    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[param_file],
        output='screen',
        condition=IfCondition(use_lidar)
    )
    
    rsx_um7_node = Node(
        package='umx_driver',
        executable='um7_driver',
        name='um7_driver',
        parameters=[param_file],
        output='screen',
        condition=IfCondition(use_imu),
        remappings=[
            ('imu/data', 'mb_imu/data'),
            ('imu/mag', 'mb_imu/mag'),
            ('imu/yaw', 'mb_imu/yaw'),
        ]
    )

    stella_ahrs_node = Node(
        package='stella_ahrs',
        executable='stella_ahrs_node',
        name='stella_ahrs_node',
        output='screen',
        parameters=[param_file],
        condition=IfCondition(use_imu),
        remappings=[
            ('imu/data', 'mb_imu/data'),
            ('imu/data_raw', 'mb_imu/data_raw'),
            ('imu/mag', 'mb_imu/mag'),
            ('imu/yaw', 'mb_imu/yaw'),
        ]
    )

    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_node',
        output='screen',
        parameters=[param_file],
        condition=IfCondition(use_imu),
        remappings=[
            ('imu/data_raw', 'mb_imu/data'),
            ('imu/mag', 'mb_imu/mag')
        ]
    )

    # rviz
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_file]
    )

    return LaunchDescription([
        declare_use_lidar_arg,
        declare_use_imu_arg,
        declare_use_rviz_arg,

        log_info,   
        robot_description_launch,
        rplidar_node,
        rsx_um7_node,
        # stella_ahrs_node,
        imu_filter_node,
        rviz2_node
    ])
