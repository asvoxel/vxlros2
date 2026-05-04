"""Launch the ASVXL camera as a managed lifecycle node.

By default the standalone executable auto-transitions through CONFIGURE → ACTIVATE.
Pass `auto_activate:=false` to stay in UNCONFIGURED and let an external lifecycle
manager (e.g. nav2_lifecycle_manager) drive the transitions.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('vxl_camera')
    default_config = os.path.join(pkg_dir, 'config', 'default.yaml')

    auto_activate_arg = DeclareLaunchArgument(
        'auto_activate', default_value='true',
        description='Auto-configure and auto-activate the lifecycle node on startup'
    )

    # Build executable args: we only pass --no-auto-activate when the user opts out.
    no_auto_args = ['--no-auto-activate']

    auto_node = Node(
        package='vxl_camera',
        executable='vxl_camera_lifecycle_node',
        name='vxl_camera',
        output='screen',
        condition=IfCondition(LaunchConfiguration('auto_activate')),
        parameters=[
            default_config,
            {
                'output_mode': LaunchConfiguration('output_mode'),
                'device_serial': LaunchConfiguration('device_serial'),
                'tf_prefix': LaunchConfiguration('tf_prefix'),
            },
        ],
    )

    manual_node = Node(
        package='vxl_camera',
        executable='vxl_camera_lifecycle_node',
        name='vxl_camera',
        output='screen',
        arguments=no_auto_args,
        condition=UnlessCondition(LaunchConfiguration('auto_activate')),
        parameters=[
            default_config,
            {
                'output_mode': LaunchConfiguration('output_mode'),
                'device_serial': LaunchConfiguration('device_serial'),
                'tf_prefix': LaunchConfiguration('tf_prefix'),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('output_mode', default_value='rgbd'),
        DeclareLaunchArgument('device_serial', default_value=''),
        DeclareLaunchArgument('tf_prefix', default_value=''),
        auto_activate_arg,
        auto_node,
        manual_node,
    ])
