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

    declare_use_rscam_arg = DeclareLaunchArgument(
        'use_rscam',
        default_value='false',
        description='Whether to use the rscam or not'
    )

    declare_use_imu_arg = DeclareLaunchArgument(
        'use_imu',
        default_value='false',
        description='Whether to use the imu or not'
    )

    use_lidar = LaunchConfiguration('use_lidar')
    use_rscam = LaunchConfiguration('use_rscam')
    use_imu = LaunchConfiguration('use_imu')    

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

    ])

    zbot_stella_n2_sensors_group_action = GroupAction([
        # lidar
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[sensor_params_file],
            output='screen',
            condition=IfCondition(use_lidar),
            remappings=[
                ('scan', 'front/scan')
            ]
        ),

        #YDLidar
        # Node(package='ydlidar',
        #     executable='ydlidar_node',
        #     name='ydlidar_node',
        #     output='screen',
        #     emulate_tty=True,
        #     parameters=[sensor_params_file],
        #     namespace='/',
        #     remappings=[
        #         ('scan', 'front/scan')
        #     ],
        #     condition=IfCondition(use_lidar)
        # ),

        #realsense_camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py']
            )),
            condition=IfCondition(use_rscam),
            launch_arguments={
                'align_depth.enable': 'true',
                'pointcloud.enable': 'true',
                # 'enable_sync': 'true',
                'rgb_camera.color_profile': '424x240x15',
                'depth_module.depth_profile': '424x240x15',
            }.items()
        ),


        # imu
        # Node(
        #     package='umx_driver',
        #     executable='um7_driver',
        #     name='um7_driver',
        #     parameters=[sensor_params_file],
        #     output='screen'
        # ),

        Node(
            package='stella_ahrs',
            executable='stella_ahrs_node',
            name='stella_ahrs_node',
            output='screen',
            parameters=[sensor_params_file],
            condition=IfCondition(use_imu),
            remappings=[
                ('imu/data', 'mw_ahrs_imu/data'),
                ('imu/data_raw', 'mw_ahrs_imu/data_raw'),
                ('imu/mag', 'mw_ahrs_imu/mag'),
                ('imu/yaw', 'mw_ahrs_imu/yaw'),
            ]
        ),
        
        # imu fiter
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_node',
            output='screen',
            parameters=[sensor_params_file],
            condition=IfCondition(use_imu), 
            remappings=[
                ('imu/data_raw', 'mw_ahrs_imu/data'),
                ('imu/mag', 'mw_ahrs_imu/mag')
            ]
        ),
    ])

    robot_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('mowbot_localization'), 'launch', 'ekf.launch.py']
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
        declare_use_rscam_arg,
        declare_use_imu_arg,
        zbot_stella_n2_bringup_group_action,
        zbot_stella_n2_sensors_group_action,
        robot_localization_launch,
        # micro_ros_agent_node
    ])