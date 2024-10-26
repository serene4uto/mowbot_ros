from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )
    
    filepath_config_twist_mux = PathJoinSubstitution(
        [FindPackageShare('mowbot_control'), 'param', 'twist_mux.yaml']
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    node_twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        remappings={('/cmd_vel_out', '/mowbot_base/cmd_vel_unstamped')},
        parameters=[
            filepath_config_twist_mux,
            {'use_sim_time': use_sim_time}
        ]
    )

    teleop_joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('mowbot_control'), 'launch', 'teleop_joy.launch.py']
        )),
    )


    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(node_twist_mux)
    ld.add_action(teleop_joy_launch)
    return ld