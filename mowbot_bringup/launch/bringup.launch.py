from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():

    extra_config_path = PathJoinSubstitution(
        [FindPackageShare('mowbot_bringup'), 'config', 'extra.yaml']
    )
    

    return LaunchDescription([  

        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace'
        ),

        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Whether to start rviz'
        ),

        DeclareLaunchArgument(
            'imu',
            default_value='false',
            description='Whether to start the imu'
        ),

        DeclareLaunchArgument(
            'laser',
            default_value='false',
            description='Whether to start the laser'
        ),

        DeclareLaunchArgument(
            name='madgwick',
            default_value='false',
            description='Use madgwick to fuse imu and magnetometer'
        ),

        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('mowbot_description'), 'launch', 'description.launch.py']
            ),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'use_sim_time': 'false'
            }.items()
        ),

        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('mowbot_bringup'), 'launch', 'twist_control.launch.py']
            ),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace')
            }.items()
        ),

        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('mowbot_bringup'), 'launch', 'sensors.launch.py']
            ),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'rviz': LaunchConfiguration('rviz'),
                'imu': LaunchConfiguration('imu'),
                'laser': LaunchConfiguration('laser')
            }.items()
        ),

        Node(
            namespace=LaunchConfiguration('namespace'),
            condition=IfCondition(LaunchConfiguration("madgwick")),
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='madgwick_filter_node',
            output='screen',
            parameters=[extra_config_path],
            remappings=[
                ('imu/data_raw', 'mb_imu/data'),
                ('imu/mag', '/mb_imu/mag')
            ]
        ),
    ])