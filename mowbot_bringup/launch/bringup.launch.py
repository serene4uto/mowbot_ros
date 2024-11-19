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

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('mowbot_bringup'), 'rviz', 'mowbot.rviz']
    )

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare('mowbot_localization'), 'config', 'ekf_params.yaml']
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
            'dcam',
            default_value='false',
            description='Whether to start the depth camera'
        ),

        DeclareLaunchArgument(
            name='madgwick',
            default_value='false',
            description='Use madgwick to fuse imu and magnetometer'
        ),

        DeclareLaunchArgument(
            name='rl',
            default_value='false',
            description='Use robot_localization'
        ),

        # DeclareLaunchArgument(
        #     name='uros_serial_port',
        #     default_value='/dev/MBB',
        #     description='Serial port for uros communication'
        # ),

        # DeclareLaunchArgument(
        #     name='uros_baudrate',
        #     default_value='115200',
        #     description='Baudrate for uros serial communication'
        # ),

        # Node(
        #     package='micro_ros_agent',
        #     executable='micro_ros_agent',
        #     name='micro_ros_agent',
        #     output='screen',
        #     arguments=['serial', '--dev', LaunchConfiguration("uros_serial_port"), '--baudrate', LaunchConfiguration("uros_baudrate")]
        # ),

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
                'imu': LaunchConfiguration('imu'),
                'laser': LaunchConfiguration('laser'),
                'dcam': LaunchConfiguration('dcam')
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

        Node(
            namespace=LaunchConfiguration('namespace'),
            condition=IfCondition(LaunchConfiguration("rl")),
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_config_path
            ]
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