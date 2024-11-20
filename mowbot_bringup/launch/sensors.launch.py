from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    imu_config_path = PathJoinSubstitution(
        [FindPackageShare('mowbot_bringup'), 'config', 'imu.yaml']
    )

    laser_config_path = PathJoinSubstitution(
        [FindPackageShare('mowbot_bringup'), 'config', 'laser.yaml']
    )

    dcam_config_path = PathJoinSubstitution(
        [FindPackageShare('mowbot_bringup'), 'config', 'rs.yaml'] 
    )

    gps_config_path = PathJoinSubstitution(
        [FindPackageShare('mowbot_bringup'), 'config', 'gps.yaml']
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
            name='dcam',
            default_value='false',
            description='Whether to start the depth camera'
        ),

        DeclareLaunchArgument(
            name='ntrip',
            default_value='false',
            description='Whether to start the ntrip client'
        ),

        DeclareLaunchArgument(
            name='gps_front',
            default_value='false',
            description='Whether to start the front gps'
        ),

        DeclareLaunchArgument(
            name='gps_rear',
            default_value='false',
            description='Whether to start the rear gps'
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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py']
            )),
            condition=IfCondition(LaunchConfiguration('dcam')),
            launch_arguments={
                'config_file': dcam_config_path
            }.items()   
        ),

        Node(
            namespace=LaunchConfiguration('namespace'),
            condition=IfCondition(LaunchConfiguration('ntrip')),
            name='ntrip_client',
            package='ntrip_client',
            executable='ntrip_ros.py',
            parameters=[gps_config_path],
        ),  

        Node(
            namespace=LaunchConfiguration('namespace'),
            condition=IfCondition(LaunchConfiguration('gps_front')),
            name='ublox_gps_front_node',
            package='ublox_gps',
            executable='ublox_gps_node',
            output='both',
            parameters=[gps_config_path]
        ),

        Node(
            namespace=LaunchConfiguration('namespace'),
            condition=IfCondition(LaunchConfiguration('gps_rear')),
            name='ublox_gps_rear_node',
            package='ublox_gps',
            executable='ublox_gps_node',
            output='both',
            parameters=[gps_config_path]
        ),
    ])