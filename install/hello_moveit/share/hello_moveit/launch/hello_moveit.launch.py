import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('franka_hybrid_control')

    # Declare launch arguments
    force_axis = LaunchConfiguration('force_axis')
    force_target = LaunchConfiguration('force_target')

    # Launch arguments
    force_axis_arg = DeclareLaunchArgument(
        'force_axis',
        default_value='2',  # Default to Z-axis
        description='Axis to control with force (0=X, 1=Y, 2=Z)'
    )

    force_target_arg = DeclareLaunchArgument(
        'force_target',
        default_value='5.0',  # Default 5N force
        description='Target force in Newtons'
    )

    # Load robot description (URDF)
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

    # Load semantic description (SRDF)
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

    # Load kinematics configuration
    kinematics_yaml = os.path.join(
        get_package_share_directory('franka_fr3_moveit_config'),
        'config', 'kinematics.yaml'
    )

    # RViz configuration
    rviz_config_file = os.path.join(
        get_package_share_directory('franka_fr3_moveit_config'),
        'config', 'moveit.rviz'
    )

    # Force position interface node
    force_position_node = Node(
        package='franka_hybrid_control',
        executable='force_position_node',
        name='force_position_interface',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            {'kinematics_config': kinematics_yaml},
            os.path.join(pkg_dir, 'config', 'force_control_params.yaml')
        ]
    )

    # Initial force axis publisher
    force_axis_pub = Node(
        package='std_msgs',
        executable='publisher',
        name='init_force_axis',
        output='screen',
        arguments=['--ros-args', '-t', '/hybrid_controller/force_axis', '-m', 'std_msgs/msg/Int32',
                   '-r', '1', '--', force_axis]
    )

    # Initial force target publisher
    force_target_pub = Node(
        package='std_msgs',
        executable='publisher',
        name='init_force_target',
        output='screen',
        arguments=['--ros-args', '-t', '/hybrid_controller/force_target', '-m', 'std_msgs/msg/Float64',
                   '-r', '1', '--', force_target]
    )

    # Start RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    return LaunchDescription([
        force_axis_arg,
        force_target_arg,
        force_position_node,
        force_axis_pub,
        force_target_pub,
        rviz_node
    ])
