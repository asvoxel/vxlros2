import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('vxl_camera')
    default_config = os.path.join(pkg_dir, 'config', 'default.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to camera config YAML file'
        ),
        DeclareLaunchArgument(
            'output_mode',
            default_value='rgbd',
            description='Output mode: rgbd, rgb+depth, ir, depth_only, color_only, all'
        ),
        DeclareLaunchArgument(
            'device_serial',
            default_value='',
            description='Device serial number (empty = auto-select)'
        ),
        DeclareLaunchArgument(
            'tf_prefix',
            default_value='',
            description='TF frame prefix for multi-camera setups'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='ROS2 logger level: debug | info | warn | error | fatal'
        ),

        Node(
            package='vxl_camera',
            executable='vxl_camera_node',
            name='vxl_camera',
            output='screen',
            arguments=['--ros-args', '--log-level',
                       ['vxl_camera:=', LaunchConfiguration('log_level')]],
            parameters=[
                LaunchConfiguration('config_file'),
                {
                    'output_mode': LaunchConfiguration('output_mode'),
                    'device_serial': LaunchConfiguration('device_serial'),
                    'tf_prefix': LaunchConfiguration('tf_prefix'),
                },
            ],
        ),
    ])
