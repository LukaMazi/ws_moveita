#!/usr/bin/env python3
# improved_hybrid_launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

def generate_launch_description():
    # Launch argumenti
    robot_ip = LaunchConfiguration('robot_ip')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    arm_id = LaunchConfiguration('arm_id')
    
    # Package paths
    franka_config = get_package_share_directory('franka_fr3_moveit_config')
    hybrid_config = get_package_share_directory('franka_hybrid_control')
    
    # Robot description
    robot_description_content = open(
        os.path.join(franka_config, 'srdf', 'fr3_no_hand.urdf.xacro')
    ).read()
    
    # Controller configuration - KLJUČNO!
    controllers_yaml = os.path.join(hybrid_config, 'config', 'hibridna_konf_kont.yaml')
    
    return LaunchDescription([
        # Argumenti
        DeclareLaunchArgument('robot_ip', description='Robot IP address'),
        DeclareLaunchArgument('arm_id', default_value='fr3'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false'),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}],
        ),
        
        # ROS2 Control Node
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                controllers_yaml,
                {'robot_description': robot_description_content},
                {'arm_id': arm_id}
            ],
            output='screen',
        ),
        
        # Load controllers z zamudo
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
                         'joint_state_broadcaster'],
                    output='screen'
                )
            ]
        ),
        
        TimerAction(
            period=4.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                         'franka_robot_state_broadcaster'],
                    output='screen',
                    condition=UnlessCondition(use_fake_hardware)
                )
            ]
        ),
        
        # KLJUČNO: Naš hibridni kontroler
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                         'cartesian_compliance_controller'],
                    output='screen'
                )
            ]
        ),
        
        # Vaš custom node za pošiljanje referenc
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='franka_hybrid_control',
                    executable='hybrid_reference_publisher',
                    name='hybrid_reference_publisher',
                    output='screen',
                )
            ]
        ),
    ])