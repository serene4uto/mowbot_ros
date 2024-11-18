from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():

    twist_control_launch_path = PathJoinSubstitution(
        [FindPackageShare('mowbot_bringup'), 'launch', 'twist_control.launch.py']
    )
    

    return LaunchDescription([  

        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace'
        ),

        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('mowbot_bringup'), 'launch', 'twist_control.launch.py']
            ),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace')
            }.items()
        ),

    ])