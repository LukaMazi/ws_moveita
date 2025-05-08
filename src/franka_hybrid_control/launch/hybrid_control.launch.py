import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import OpaqueFunction

def launch_setup(context, *args, **kwargs):
    # Get the package directory (this package)
    pkg_dir = get_package_share_directory('franka_hybrid_control')

    # Get the robot_ip from the launch argument
    robot_ip = LaunchConfiguration('robot_ip').perform(context) # Use perform to get the string value

    # Get the path to the main Franka MoveIt launch file
    # Confirmed package name is 'franka_fr3_moveit_config' and file is 'moveit.launch.py'
    try:
        franka_moveit_config_share = get_package_share_directory('franka_fr3_moveit_config') # Corrected package name
        moveit_launch_file = os.path.join(franka_moveit_config_share, 'launch', 'moveit.launch.py') # <--- Confirmed filename

        if not os.path.exists(moveit_launch_file):
             print("!!! Error: Confirmed MoveIt launch file not found at the expected location. !!!")
             print(f"!!! Looked for: {moveit_launch_file} !!!")
             print("!!! Please ensure 'franka_fr3_moveit_config' is installed and sourced correctly. !!!")
             return []

        print(f"--- Including main MoveIt launch file: {moveit_launch_file} ---") # Debug print

    except Exception as e:
        print(f"!!! Error finding franka_fr3_moveit_config package share directory: {e} !!!")
        print("!!! Please ensure 'franka_fr3_moveit_config' is installed and sourced. !!!")
        return [] # Return empty list if dependency not found

    # Declare launch arguments (already present, but needed to pass them around)
    # These arguments are used both at the top level and passed as parameters
    force_axis = LaunchConfiguration('force_axis')
    force_target = LaunchConfiguration('force_target')

    # Use LaunchConfiguration to access the values of the arguments for the node parameters
    force_axis_value = LaunchConfiguration('force_axis')
    force_target_value = LaunchConfiguration('force_target')

    # Include the main Franka MoveIt launch file
    # This will start the robot driver, state publishers, controllers, and MoveIt nodes
    franka_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([moveit_launch_file]),
        # Pass arguments required by the included launch file
        launch_arguments=[
            ('robot_ip', robot_ip), # Pass the robot_ip argument
            # Add other arguments required by the included launch file here, if any
            # ('use_sim_time', 'false'), # Example if you need to pass sim time
            # Check franka_fr3_moveit_config/launch/moveit.launch.py for required arguments
        ]
    )

    # Force position interface node
    # This node will receive force_axis and force_target as parameters
    force_position_node = Node(
        package='franka_hybrid_control',
        executable='force_position_node',
        name='force_position_interface',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'force_control_params.yaml'),
            {'force_axis': force_axis_value},  # Pass force_axis launch arg as a parameter
            {'force_target': force_target_value} # Pass force_target launch arg as a parameter
            # Note: force_kp, force_ki, force_kd should be in force_control_params.yaml
            # If FrankaHybridController is a separate Node, it needs its params loaded elsewhere
        ],
        # Add remappings if necessary for your node to find topics/services under namespaces
        # remappings=[
        #     ('/tf', '/tf'), # Example remapping if needed
        # ]
    )

    # Removed the incorrect 'std_msgs' publisher nodes

    # Return the LaunchDescription with the included launch file FIRST
    # and then your custom node
    return [
        franka_moveit_launch, # Include this action BEFORE your node
        force_position_node,
        # The parameters for force_axis and force_target are now part of the force_position_node definition
    ]


def generate_launch_description():
    # Declare the launch arguments at the top level
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        description='IP address of the robot'
    )

    force_axis_arg = DeclareLaunchArgument(
        'force_axis',
        default_value='2',  # Default to Z-axis (index 2)
        description='Axis index to control with force (0=X, 1=Y, 2=Z)'
    )

    force_target_arg = DeclareLaunchArgument(
        'force_target',
        default_value='5.0',  # Default 5N force in Newtons
        description='Target force in Newtons for the specified axis'
    )

    # Use OpaqueFunction to run the launch_setup logic, which finds and includes the MoveIt launch file
    # This allows handling potential errors during the setup phase.
    return LaunchDescription([
        robot_ip_arg, # Include the argument definitions
        force_axis_arg,
        force_target_arg,
        OpaqueFunction(function=launch_setup), # This function returns the list of actions (included launch + your node)
    ])