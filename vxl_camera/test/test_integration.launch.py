"""
Integration test launch file.
Starts VxlCameraNode and verifies topics are published.
Requires a connected VxlSense device.
"""
import os
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest

import launch_testing
import rclpy
from rclpy.node import Node as RclpyNode
from sensor_msgs.msg import Image
from vxl_camera_msgs.msg import RGBD


def generate_test_description():
    pkg_dir = get_package_share_directory('vxl_camera')
    config = os.path.join(pkg_dir, 'config', 'default.yaml')

    camera_node = Node(
        package='vxl_camera',
        executable='vxl_camera_node',
        name='vxl_camera_test',
        parameters=[config, {'output_mode': 'rgbd'}],
        output='screen',
    )

    return LaunchDescription([
        camera_node,
        ReadyToTest(),
    ]), {'camera_node': camera_node}


class TestVxlCameraIntegration(unittest.TestCase):
    """Integration tests - require a connected VxlSense device."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = RclpyNode('test_integration')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_rgbd_topic_published(self):
        """Verify RGBD topic is published within 10 seconds."""
        received = []

        sub = self.node.create_subscription(
            RGBD, '/vxl_camera_test/rgbd', lambda msg: received.append(msg), 10)

        end_time = self.node.get_clock().now() + rclpy.duration.Duration(seconds=10)
        while self.node.get_clock().now() < end_time and len(received) == 0:
            rclpy.spin_once(self.node, timeout_sec=0.5)

        sub  # keep reference
        self.assertGreater(len(received), 0, "No RGBD messages received within 10s")

        msg = received[0]
        self.assertGreater(msg.rgb.width, 0)
        self.assertGreater(msg.depth.width, 0)
        self.assertEqual(msg.rgb.encoding, "bgr8")
        self.assertEqual(msg.depth.encoding, "16UC1")

    def test_extrinsics_published(self):
        """Verify extrinsics topic is published (latched)."""
        from vxl_camera_msgs.msg import Extrinsics

        received = []
        sub = self.node.create_subscription(
            Extrinsics, '/vxl_camera_test/extrinsics/depth_to_color',
            lambda msg: received.append(msg),
            rclpy.qos.QoSProfile(
                depth=1,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))

        end_time = self.node.get_clock().now() + rclpy.duration.Duration(seconds=5)
        while self.node.get_clock().now() < end_time and len(received) == 0:
            rclpy.spin_once(self.node, timeout_sec=0.5)

        sub  # keep reference
        self.assertGreater(len(received), 0, "No Extrinsics message received")
        self.assertEqual(len(received[0].rotation), 9)
        self.assertEqual(len(received[0].translation), 3)
