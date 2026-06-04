from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('demos_pca9685_hw_controller')
    urdf_path = os.path.join(pkg, 'urdf', 'robot_with_pca9685.urdf')

    with open(urdf_path, 'r', encoding='utf-8') as f:
        urdf_content = f.read()

    # ✅ FORCER le type string explicite
    robot_description = ParameterValue(urdf_content, value_type=str)

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            output='screen',
            parameters=[
                {'update_rate': 50},
                os.path.join(pkg, 'config', 'controllers.yaml')
            ]
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['forward_position_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        ),
    ])