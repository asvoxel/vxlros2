"""Multi-camera launch — separate processes (independent crash isolation).

Each camera runs in its own process with its own ASVXL Context. Use this for
maximum isolation; for intra-process zero-copy use multi_camera_composable.launch.py.

Each node binds to a specific device by serial number and namespaces all topics
under /camera_N/* with TF prefix camN_.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('vxl_camera')
    default_config = os.path.join(pkg_dir, 'config', 'default.yaml')

    nodes = []
    for idx in (1, 2):
        nodes.append(Node(
            package='vxl_camera',
            executable='vxl_camera_lifecycle_node',
            name=f'vxl_camera_{idx}',
            namespace=f'camera_{idx}',
            output='screen',
            parameters=[
                default_config,
                {
                    'device_serial': LaunchConfiguration(f'serial_{idx}'),
                    'tf_prefix': f'cam{idx}_',
                },
            ],
        ))

    return LaunchDescription([
        DeclareLaunchArgument('serial_1', default_value='',
                              description='Serial number of camera 1'),
        DeclareLaunchArgument('serial_2', default_value='',
                              description='Serial number of camera 2'),
        *nodes,
    ])
