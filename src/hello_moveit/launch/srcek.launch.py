import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Load robot description
    xacro_file = os.path.join(
        get_package_share_directory('franka_fr3_moveit_config'),
        'srdf', 'fr3_no_hand.urdf.xacro'
    )
    robot_description = {
        'robot_description': ParameterValue(
            Command([FindExecutable(name='xacro'), ' ', xacro_file]),
            value_type=str
        )
    }
    
    # Load semantic description
    srdf_xacro = os.path.join(
        get_package_share_directory('franka_fr3_moveit_config'),
        'srdf', 'fr3_arm.srdf.xacro'
    )
    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(
            Command([FindExecutable(name='xacro'), ' ', srdf_xacro]),
            value_type=str
        )
    }
    
    # Optional: kinematics, planning settings, etc.
    return LaunchDescription([
        Node(
            package='hello_moveit',
            executable='srcek',
            output='screen',
            parameters=[
                robot_description,
                robot_description_semantic,
            ],
        )
    ])
