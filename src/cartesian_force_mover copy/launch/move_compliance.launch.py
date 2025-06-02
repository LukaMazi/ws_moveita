from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('cartesian_force_mover_copy'),
        'config',
        'compliance_controller.yaml'
    )

    return LaunchDescription([
        Node(
            package='cartesian_force_mover_copy',
            executable='bare_mover',
            name='bare_mover',
            parameters=[config_path],
            output='screen'
        )
    ])