# hybrid_control.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, file_path)
    with open(absolute_path, 'r') as file:
        return yaml.safe_load(file)

def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware', default='false')
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands', default='false')
    arm_id = LaunchConfiguration('arm_id', default='fr3')

    franka_config = get_package_share_directory('franka_fr3_moveit_config')
    hybrid_config = get_package_share_directory('franka_hybrid_control')

    urdf_xacro = os.path.join(franka_config, 'srdf', 'fr3_no_hand.urdf.xacro')
    srdf_xacro = os.path.join(franka_config, 'srdf', 'fr3_arm.srdf.xacro')
    rviz_config = os.path.join(franka_config, 'rviz', 'moveit.rviz')

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                FindExecutable(name='xacro'), ' ', urdf_xacro,
                ' hand:=false',
                ' robot_ip:=', robot_ip,
                ' arm_id:=', arm_id,
                ' use_fake_hardware:=', use_fake_hardware,
                ' fake_sensor_commands:=', fake_sensor_commands,
                ' ros2_control:=true'
            ]),
            value_type=str
        )
    }

    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(
            Command([
                FindExecutable(name='xacro'), ' ', srdf_xacro,
                ' hand:=false'
            ]),
            value_type=str
        )
    }

    # Load configuration
    kinematics_yaml = load_yaml('franka_fr3_moveit_config', 'config/kinematics.yaml')
    ompl_yaml = load_yaml('franka_fr3_moveit_config', 'config/ompl_planning.yaml')
    moveit_simple_controllers_yaml = load_yaml('franka_fr3_moveit_config', 'config/fr3_controllers.yaml')
    force_params_yaml = load_yaml('franka_hybrid_control', 'config/force_control_params.yaml')

    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_pipeline_config['move_group'].update(ompl_yaml)

    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
    )

    force_position_node = Node(
        package='franka_hybrid_control',
        executable='force_position_node',
        name='force_position_node',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            force_params_yaml,
        ],
    )

    custom_publisher_node = Node(
        package='franka_hybrid_control',
        executable='custom_publisher',
        name='custom_publisher',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', description='Robot IP address'),
        DeclareLaunchArgument('arm_id', default_value='fr3', description='Robot arm ID'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false'),
        DeclareLaunchArgument('fake_sensor_commands', default_value='false'),
        
        # Then launch the MoveIt nodes
        move_group_node,
        rviz_node,
        force_position_node,
        custom_publisher_node,
    ])