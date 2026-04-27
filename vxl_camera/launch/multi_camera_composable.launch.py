"""Multi-camera launch — single process via ComposableNodeContainer.

Loads all cameras as components inside one container, enabling intra-process
zero-copy message passing for downstream consumers in the same container.

Container choice: `component_container_isolated` runs each component on its own
single-threaded executor in a dedicated thread. The SDK's polling thread per
camera blocks on waitForFrameSet(); using `_isolated` keeps each camera's
callbacks from blocking the others.

Each ComposableNode here is the non-lifecycle variant (vxl_camera::VxlCameraNode)
because lifecycle nodes inside ComposableNodeContainer require an external
lifecycle manager — out of scope for this convenience launch.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_dir = get_package_share_directory('vxl_camera')
    default_config = os.path.join(pkg_dir, 'config', 'default.yaml')

    components = []
    for idx in (1, 2):
        components.append(ComposableNode(
            package='vxl_camera',
            plugin='vxl_camera::VxlCameraNode',
            name=f'vxl_camera_{idx}',
            namespace=f'camera_{idx}',
            parameters=[
                default_config,
                {
                    'device_serial': LaunchConfiguration(f'serial_{idx}'),
                    'tf_prefix': f'cam{idx}_',
                },
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
        ))

    container = ComposableNodeContainer(
        name='vxl_cameras_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_isolated',
        composable_node_descriptions=components,
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('serial_1', default_value='',
                              description='Serial number of camera 1'),
        DeclareLaunchArgument('serial_2', default_value='',
                              description='Serial number of camera 2'),
        container,
    ])
