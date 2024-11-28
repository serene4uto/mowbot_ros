from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml





def generate_launch_description():

    nav2_params = PathJoinSubstitution(
        [FindPackageShare('mowbot_navigation'), 'config', 'nav_no_map_params.yaml']
    )
    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key="", param_rewrites="", convert_types=True
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            "rviz",
            default_value="False",
            description="Whether to launch RViz"
        ),

        DeclareLaunchArgument(
            "tf",
            default_value="True",
            description="Whether to launch TF"
        ),

        # nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']
                )
            ),
            launch_arguments={
                "use_sim_time": "False",
                "params_file": configured_params,
                "autostart": "True",
            }.items(),
        ),

        # rviz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('mowbot_navigation'), 'launch', 'rviz.launch.py']
                )
            ),
            condition=IfCondition(LaunchConfiguration("rviz"))
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            condition=IfCondition(LaunchConfiguration("tf"))
        )

    ])