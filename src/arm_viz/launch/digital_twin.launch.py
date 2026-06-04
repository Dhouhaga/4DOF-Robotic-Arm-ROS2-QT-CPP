from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    urdf_file = PathJoinSubstitution([
        FindPackageShare('arm_viz'), 'urdf', 'arm.urdf'
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare('arm_viz'), 'config', 'arm.rviz'
    ])

    robot_description = ParameterValue(
        Command([FindExecutable(name='cat'), ' ', urdf_file]),
        value_type=str
    )

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        Node(
            package='arm_viz',
            executable='joint_state_bridge',
            name='joint_state_bridge',
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),

    ])