# Global to Polar Converter

This ROS 2 package converts global waypoints to polar coordinates and logs sensor data for machine learning applications.

## Overview

The package consists of two main nodes:
- **global_to_polar_node**: Converts CSV waypoints to polar grid format
- **data_logger_node**: Records sensor data (laser scan, polar grid, path) to CSV files

## Dependencies

- ROS 2 Humble
- rclcpp
- nav_msgs
- sensor_msgs
- tf2
- planning_custom_msgs
- ament_index_cpp

## Building

```bash
colcon build --packages-select global_to_polar_cpp
source install/setup.bash
```

## Configuration

### Global to Polar Node
Edit `config/global_to_polar_params.yaml`:
```yaml
global_to_polar_node:
  ros__parameters:
    path_csv_file: "$(find-pkg-share global_to_polar_cpp)/line/slam_tool_box.csv"
    lookahead_points: 20
    search_window: 10
```

### Data Logger Node
Edit `config/data_logger_params.yaml`:
```yaml
data_logger_node:
  ros__parameters:
    map_name: "Spielberg_map"
    auto_save_dir: "auto_save"
    enable_auto_filename: true
```

## Usage

### Launch Both Nodes
```bash
# With data logging enabled
ros2 launch global_to_polar_cpp global_to_polar_complete.launch.py enable_data_logger:=true

# With custom map name
ros2 launch global_to_polar_cpp global_to_polar_complete.launch.py enable_data_logger:=true map_name:=custom_track

# With custom waypoint file
ros2 launch global_to_polar_cpp global_to_polar_complete.launch.py path_csv_file:=/path/to/waypoints.csv
```

### Launch Individual Nodes
```bash
# Global to polar converter only
ros2 launch global_to_polar_cpp global_to_polar.launch.py

# Data logger only
ros2 launch global_to_polar_cpp data_logger.launch.py map_name:=test_track
```

## Data Output

Data files are automatically saved to `src/global_to_polar_cpp/auto_save/` with the format:
```
mapname_YYYY-MM-DD_HH-MM-SS-mmm.csv
```

### CSV Format
Each row contains:
- 1080 laser scan values (scan_0 to scan_1079)
- 1080 polar grid values (grid_0 to grid_1079) 
- 16 path points with x, y, velocity, yaw (x1,y1,v1,yaw1 to x16,y16,v16,yaw16)

## Topics

### Subscribed
- `/pf/pose/odom` (nav_msgs/Odometry) - Robot pose
- `/scan` (sensor_msgs/LaserScan) - Laser scan data
- `/planned_path_with_velocity` (planning_custom_msgs/PathWithVelocity) - Path data

### Published
- `/polar_grid` (global_to_polar_cpp/PolarGrid) - Polar coordinate grid

## File Structure

```
src/global_to_polar_cpp/
├── auto_save/           # Auto-generated data files
├── config/              # Configuration files
├── launch/              # Launch files
├── line/                # Waypoint CSV files
├── msg/                 # Custom message definitions
└── src/                 # Source code
```

## Parameters

### global_to_polar_node
- `path_csv_file`: Path to waypoint CSV file
- `lookahead_points`: Number of forward waypoints to process (default: 20)
- `search_window`: Search window size for closest waypoint (default: 10)

### data_logger_node
- `map_name`: Map identifier for filename generation
- `auto_save_dir`: Output directory name
- `enable_auto_filename`: Enable automatic filename generation
- `precision`: Floating point precision for CSV output (default: 5)