from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('simple_cartesian_force_demo'),
        'config',
        'compliance_controller.yaml'  # make sure spelling is correct
    )

    return LaunchDescription([
        Node(
            package='simple_cartesian_force_demo',
            executable='simple_publisher',
            output='screen',
            parameters=[config_path]  # <- pass YAML parameters here
        )
    ])
