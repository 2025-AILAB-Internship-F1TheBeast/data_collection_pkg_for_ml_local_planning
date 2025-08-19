from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('global_to_polar_cpp')
    
    # Default config file path
    default_config_file = os.path.join(pkg_dir, 'config', 'data_logger_params.yaml')
    
    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to the config file'
    )
    
    # Read config file to get default map name
    default_config_file = os.path.join(pkg_dir, 'config', 'data_logger_params.yaml')
    
    # Try source directory first (for development)
    src_config_file = os.path.join(os.getcwd(), 'src', 'global_to_polar_cpp', 'config', 'data_logger_params.yaml')
    config_file_to_read = src_config_file if os.path.exists(src_config_file) else default_config_file
    
    # Extract default map name from config file
    try:
        with open(config_file_to_read, 'r') as f:
            config_dict = yaml.safe_load(f)
            config_map_name = config_dict.get('data_logger_node', {}).get('ros__parameters', {}).get('map_name', 'default_map')
    except Exception:
        config_map_name = 'default_map'
    
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value=config_map_name,
        description='Map name for automatic file naming'
    )
    
    enable_auto_filename_arg = DeclareLaunchArgument(
        'enable_auto_filename',
        default_value='true',
        description='Enable automatic filename generation (map_name + timestamp)'
    )
    
    # Data Logger Node
    data_logger_node = Node(
        package='global_to_polar_cpp',
        executable='data_logger_node',
        name='data_logger_node',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'map_name': LaunchConfiguration('map_name'),
                'enable_auto_filename': LaunchConfiguration('enable_auto_filename')
            }
        ],
        output='screen'
    )
    
    return LaunchDescription([
        config_file_arg,
        map_name_arg,
        enable_auto_filename_arg,
        data_logger_node
    ])