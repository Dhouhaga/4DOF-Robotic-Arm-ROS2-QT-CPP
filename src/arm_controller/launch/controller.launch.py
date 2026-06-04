from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arm_viz_launch = PathJoinSubstitution([
        FindPackageShare('arm_viz'), 'launch', 'digital_twin.launch.py'
    ])

    return LaunchDescription([
        Node(
            package='arm_controller',
            executable='command_router',
            name='command_router',
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(arm_viz_launch),
        ),
    ])
