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
    
    sensor_params_dir = os.path.join(get_package_share_directory('mowbot_demo'), 'param')
    sensor_params_file = os.path.join(sensor_params_dir, 'sensor_params.yaml')
    ekf_params_file = os.path.join(get_package_share_directory('mowbot_localization'), 'param', 'ekf_params.yaml')
    rviz_file = os.path.join(get_package_share_directory('mowbot_demo'), 'rviz', 'sensors.rviz')  
    rs_config_file = os.path.join(sensor_params_dir, 'rs_config.yaml')

    declare_use_lidar_arg = DeclareLaunchArgument(
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

    declare_use_lidar_3d_flash_arg = DeclareLaunchArgument(
        'use_lidar_3d_flash',
        default_value='false',
        description='Whether to use the 3d flash lidar or not'
    )

    declare_use_rl_arg = DeclareLaunchArgument(
        'use_rl',
        default_value='false',
        description='Whether to use the robot_localization or not'
    )

    declare_use_uros_agent_arg = DeclareLaunchArgument(
        'use_uros_agent',
        default_value='false',
        description='Whether to use the micro_ros_agent or not'
    )

    declare_use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to start RViz or not'
    )

    use_lidar = LaunchConfiguration('use_lidar')
    use_rscam = LaunchConfiguration('use_rscam')
    use_imu = LaunchConfiguration('use_imu')  
    use_lidar_3d_flash = LaunchConfiguration('use_lidar_3d_flash')

    use_rl = LaunchConfiguration('use_rl')
    use_uros_agent = LaunchConfiguration('use_uros_agent')  

    use_rviz = LaunchConfiguration('use_rviz')

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
                # ('scan', 'front/scan')
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
                # 'align_depth.enable': 'true',
                # 'pointcloud.enable': 'true',
                # # 'enable_sync': 'true',
                # 'rgb_camera.color_profile': '424x240x15',
                # 'depth_module.depth_profile': '424x240x15',
                'config_file': rs_config_file
            }.items()
        ),

        # 3d flash lidar
        Node(
            package='roboscan_nsl2206',
            executable='roboscan_publish_node',
            output='screen',
            parameters=[
                {"0. cvShow": False},
            ],
            condition=IfCondition(use_lidar_3d_flash)
        ),


        # imu
        Node(
            package='umx_driver',
            executable='um7_driver',
            name='um7_driver',
            parameters=[sensor_params_file],
            output='screen',
            condition=IfCondition(use_imu),
            remappings=[
                ('imu/data', 'mb_imu/data'),
                ('imu/mag', 'mb_imu/mag'),
                ('imu/yaw', 'mb_imu/yaw'),
            ]
        ),

        # Node(
        #     package='stella_ahrs',
        #     executable='stella_ahrs_node',
        #     name='stella_ahrs_node',
        #     output='screen',
        #     parameters=[sensor_params_file],
        #     condition=IfCondition(use_imu),
        #     remappings=[
        #         ('imu/data', 'mw_ahrs_imu/data'),
        #         ('imu/data_raw', 'mw_ahrs_imu/data_raw'),
        #         ('imu/mag', 'mw_ahrs_imu/mag'),
        #         ('imu/yaw', 'mw_ahrs_imu/yaw'),
        #     ]
        # ),
        
        # imu fiter
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_node',
            output='screen',
            parameters=[sensor_params_file],
            condition=IfCondition(use_imu),
            remappings=[
                ('imu/data_raw', 'mb_imu/data'),
                ('imu/mag', 'mb_imu/mag')
            ]
        ),
    ])

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file],
        condition=IfCondition(use_rl)
    )
    
    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=["serial", "--dev", "/dev/mbb", "-v6"],
        condition=IfCondition(use_uros_agent)
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
        # Declare arguments
        declare_use_lidar_arg,
        declare_use_rscam_arg,
        declare_use_imu_arg,
        declare_use_lidar_3d_flash_arg,
        declare_use_rl_arg,
        declare_use_uros_agent_arg,
        declare_use_rviz_arg,

        # Group actions
        zbot_stella_n2_bringup_group_action,
        zbot_stella_n2_sensors_group_action,
        robot_localization_node,
        micro_ros_agent_node,
        rviz2_node
    ])