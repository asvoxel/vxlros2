"""Launch a VXL615 (VXL6X5 family) with product-tuned defaults.

Loads `config/vxl615.yaml` (depth_scale=8.0, device-side denoise/median/outlier
enabled by default) and runs the lifecycle node with auto-activate.

Compared to vxl435.launch.py:
  - align_depth.scale defaults to 8.0
  - filters.device.* are enabled (zero host CPU; runs on the FPGA)
  - filters.host.* are off (avoid double-filtering on already-clean depth)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('vxl_camera')
    default_config = os.path.join(pkg, 'config', 'vxl615.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='Path to YAML parameter file'
        ),
        DeclareLaunchArgument(
            'device_serial', default_value='',
            description='Serial number; empty = auto-select first VXL615'
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
