"""Launch a VXL435 with product-tuned defaults.

Loads `config/vxl435.yaml` (raw=mm depth scale, no device-side filters) and
runs the lifecycle node with auto-activate. For overriding individual values,
either:
  - Pass `--ros-args -p key:=val` after the launch invocation
  - Or copy the YAML to your own workspace and pass `config_file:=/path/to/your.yaml`
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('vxl_camera')
    default_config = os.path.join(pkg, 'config', 'vxl435.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='Path to YAML parameter file'
        ),
        DeclareLaunchArgument(
            'device_serial', default_value='',
            description='Serial number; empty = auto-select first VXL435'
        ),
        DeclareLaunchArgument(
            'tf_prefix', default_value='',
            description='TF frame prefix (use camN_ for multi-camera setups)'
        ),
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='ROS logger level: debug | info | warn | error'
        ),

        Node(
            package='vxl_camera',
            executable='vxl_camera_lifecycle_node',
            name='vxl_camera',
            output='screen',
            arguments=['--ros-args', '--log-level',
                       ['vxl_camera:=', LaunchConfiguration('log_level')]],
            parameters=[
                LaunchConfiguration('config_file'),
                {
                    'device_serial': LaunchConfiguration('device_serial'),
                    'tf_prefix': LaunchConfiguration('tf_prefix'),
                },
            ],
        ),
    ])
