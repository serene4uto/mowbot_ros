from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    imu_config_path = PathJoinSubstitution(
        [FindPackageShare('mowbot_bringup'), 'config', 'imu.yaml']
    )

    laser_config_path = PathJoinSubstitution(
        [FindPackageShare('mowbot_bringup'), 'config', 'laser.yaml']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('mowbot_bringup'), 'rviz', 'sensors.rviz']
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace'
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
            'rviz',
            default_value='false',
            description='Whether to start rviz'
        ),

        Node(
            namespace = LaunchConfiguration('namespace'),
            name = 'witmotion',
            package = 'witmotion_ros',
            executable = 'witmotion_ros_node',
            parameters = [imu_config_path],
            output = 'screen',
            remappings = [
                ('/imu', '/mb_imu/data'),
                ('/magnetometer', 'mb_imu/mag'),
            ],
            condition=IfCondition(LaunchConfiguration('imu'))
        ),

        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[laser_config_path],
            output='screen',
            condition=IfCondition(LaunchConfiguration('laser'))
        ),

        Node(
            namespace = LaunchConfiguration('namespace'),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', rviz_config_path]
        ),

    ])