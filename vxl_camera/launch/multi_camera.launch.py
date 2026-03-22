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
        DeclareLaunchArgument('serial_1', default_value='',
                              description='Serial number of camera 1'),
        DeclareLaunchArgument('serial_2', default_value='',
                              description='Serial number of camera 2'),

        Node(
            package='vxl_camera',
            executable='vxl_camera_node',
            name='vxl_camera_1',
            namespace='camera_1',
            output='screen',
            parameters=[
                default_config,
                {
                    'device_serial': LaunchConfiguration('serial_1'),
                    'tf_prefix': 'cam1_',
                },
            ],
        ),

        Node(
            package='vxl_camera',
            executable='vxl_camera_node',
            name='vxl_camera_2',
            namespace='camera_2',
            output='screen',
            parameters=[
                default_config,
                {
                    'device_serial': LaunchConfiguration('serial_2'),
                    'tf_prefix': 'cam2_',
                },
            ],
        ),
    ])
