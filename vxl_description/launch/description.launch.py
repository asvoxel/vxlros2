import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('vxl_description')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'vxl_camera.urdf.xacro')

    return LaunchDescription([
        DeclareLaunchArgument('tf_prefix', default_value=''),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': Command([
                    'xacro ', xacro_file,
                    ' tf_prefix:=', LaunchConfiguration('tf_prefix'),
                ]),
            }],
        ),
    ])
