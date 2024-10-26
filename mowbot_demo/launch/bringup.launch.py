from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import os



def generate_launch_description():
    
    sensor_params_file = os.path.join(get_package_share_directory('mowbot_demo'), 'param', 'sensor_params.yaml')
    ekf_params_file = os.path.join(get_package_share_directory('mowbot_demo'), 'param', 'ekf_params.yaml')  

    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar',
        default_value='false',
        description='Whether to use the lidar or not'
    )

    declare_use_arg = DeclareLaunchArgument(
        'use_rscam',
        default_value='false',
        description='Whether to use the rscam or not'
    )

    use_lidar = LaunchConfiguration('use_lidar')
    use_rscam = LaunchConfiguration('use_rscam')

    zbot_stella_n2_bringup_group_action = GroupAction([

        # Description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('mowbot_description'), 'launch', 'description.launch.py']
            ))
        ),

        # Base Teleop
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('mowbot_control'), 'launch', 'teleop_base.launch.py']
            ))
        ),

        # Joy Teleop
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(PathJoinSubstitution(
        #         [FindPackageShare('mowbot_control'), 'launch', 'teleop_joy.launch.py']
        #     ))
        # ),

    ])

    zbot_stella_n2_sensors_group_action = GroupAction([
        # lidar
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[sensor_params_file],
            output='screen',
            condition=IfCondition(use_lidar)
        ),
        # imu
        Node(
            package='umx_driver',
            executable='um7_driver',
            name='um7_driver',
            parameters=[sensor_params_file],
            output='screen'
        ),
        
        # imu fiter
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_node',
            output='screen',
            parameters=[sensor_params_file],
            remappings=[
                ('imu/data_raw', 'mw_ahrs_imu/data'),
                ('imu/mag', 'mw_ahrs_imu/mag')
            ]
        ),
    ])

    robot_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('mowbot_demo'), 'launch', 'ekf.launch.py']
            ))
    )
    
    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=["serial", "--dev", "/dev/mbb", "-v6"]
    ),
    


    return LaunchDescription([
        use_lidar_arg,
        declare_use_arg,
        zbot_stella_n2_bringup_group_action,
        # zbot_stella_n2_sensors_group_action,
        # robot_localization_launch,
        # micro_ros_agent_node
    ])