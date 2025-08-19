from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('global_to_polar_cpp')
    
    # Default config file path
    default_config_file = os.path.join(pkg_dir, 'config', 'global_to_polar_params.yaml')
    
    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to the config file'
    )
    
    path_csv_file_arg = DeclareLaunchArgument(
        'path_csv_file',
        default_value=os.path.join(pkg_dir, 'line', 'slam_tool_box.csv'),
        description='Path to the CSV file containing waypoints'
    )
    
    # Global to Polar Node
    global_to_polar_node = Node(
        package='global_to_polar_cpp',
        executable='global_to_polar_node',
        name='global_to_polar_node',
        parameters=[
            LaunchConfiguration('config_file'),
            {'path_csv_file': LaunchConfiguration('path_csv_file')}
        ],
        output='screen'
    )
    
    return LaunchDescription([
        config_file_arg,
        path_csv_file_arg,
        global_to_polar_node
    ])