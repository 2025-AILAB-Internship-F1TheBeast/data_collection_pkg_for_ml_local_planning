from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch import conditions
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('global_to_polar_cpp')
    
    # Launch arguments
    enable_data_logger_arg = DeclareLaunchArgument(
        'enable_data_logger',
        default_value='false',
        description='Whether to launch the data logger node'
    )
    
    global_to_polar_config_arg = DeclareLaunchArgument(
        'global_to_polar_config',
        default_value=os.path.join(pkg_dir, 'config', 'global_to_polar_params.yaml'),
        description='Config file for global to polar node'
    )
    
    data_logger_config_arg = DeclareLaunchArgument(
        'data_logger_config',
        default_value=os.path.join(pkg_dir, 'config', 'data_logger_params.yaml'),
        description='Config file for data logger node'
    )
    
    path_csv_file_arg = DeclareLaunchArgument(
        'path_csv_file',
        default_value=os.path.join(pkg_dir, 'line', 'slam_tool_box.csv'),
        description='Path to the CSV file containing waypoints'
    )
    
    # Read data logger config file to get default map name
    data_logger_config_file = os.path.join(pkg_dir, 'config', 'data_logger_params.yaml')
    
    # Try source directory first (for development)
    src_data_logger_config = os.path.join(os.getcwd(), 'src', 'global_to_polar_cpp', 'config', 'data_logger_params.yaml')
    config_file_to_read = src_data_logger_config if os.path.exists(src_data_logger_config) else data_logger_config_file
    
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
    
    # Global to Polar Node
    global_to_polar_node = Node(
        package='global_to_polar_cpp',
        executable='global_to_polar_node',
        name='global_to_polar_node',
        parameters=[
            LaunchConfiguration('global_to_polar_config'),
            {'path_csv_file': LaunchConfiguration('path_csv_file')}
        ],
        output='screen'
    )
    
    # Data Logger Node (conditionally launched)
    data_logger_node = Node(
        package='global_to_polar_cpp',
        executable='data_logger_node',
        name='data_logger_node',
        parameters=[
            LaunchConfiguration('data_logger_config'),
            {
                'map_name': LaunchConfiguration('map_name'),
                'enable_auto_filename': LaunchConfiguration('enable_auto_filename')
            }
        ],
        output='screen',
        condition=conditions.IfCondition(LaunchConfiguration('enable_data_logger'))
    )
    
    return LaunchDescription([
        enable_data_logger_arg,
        global_to_polar_config_arg,
        data_logger_config_arg,
        path_csv_file_arg,
        map_name_arg,
        enable_auto_filename_arg,
        global_to_polar_node,
        data_logger_node
    ])