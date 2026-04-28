"""End-to-end launch test for VxlCameraLifecycleNode using a mock backend.

Launches the test executable `vxl_camera_lifecycle_mock` (built only under
BUILD_TESTING) which wraps the lifecycle node with MockCameraBackend + a
synthetic frame pump. Verifies real ROS2 topic publication, lifecycle service
responses, and parameter callback over the wire — without any hardware.
"""
import unittest

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest

import rclpy
from rclpy.node import Node as RclpyNode
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data

from vxl_camera_msgs.msg import Metadata, Extrinsics
from diagnostic_msgs.msg import DiagnosticArray
from lifecycle_msgs.srv import GetState


def generate_test_description():
    node = Node(
        package='vxl_camera',
        executable='vxl_camera_lifecycle_mock',
        name='vxl_camera',
        output='screen',
    )
    return LaunchDescription([node, ReadyToTest()]), {'node': node}


class TestLifecycleLaunch(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = RclpyNode('test_lifecycle_launch_client')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, predicate, timeout_s=10.0):
        end = self.node.get_clock().now() + rclpy.duration.Duration(seconds=timeout_s)
        while self.node.get_clock().now() < end and not predicate():
            rclpy.spin_once(self.node, timeout_sec=0.2)

    def test_lifecycle_state_is_active(self):
        client = self.node.create_client(GetState, '/vxl_camera/get_state')
        self.assertTrue(client.wait_for_service(timeout_sec=5.0),
            'lifecycle get_state service did not appear')
        future = client.call_async(GetState.Request())
        self._spin_until(lambda: future.done(), timeout_s=5.0)
        self.assertTrue(future.done(), 'get_state did not return')
        # PRIMARY_STATE_ACTIVE = 3
        self.assertEqual(future.result().current_state.id, 3,
            f'Expected ACTIVE (3), got {future.result().current_state.label}')

    def test_color_metadata_topic_published(self):
        # Publisher uses SensorDataQoS (best-effort); subscriber must match
        # else QoS-incompatible and no messages flow.
        received = []
        sub = self.node.create_subscription(
            Metadata, '/vxl_camera/color/metadata',
            lambda m: received.append(m), qos_profile_sensor_data)
        # Synthetic source pumps at ~30 Hz; expect several messages within seconds.
        self._spin_until(lambda: len(received) >= 3, timeout_s=5.0)
        sub  # keep alive
        self.assertGreaterEqual(len(received), 3,
            f'Expected ≥3 color metadata messages, got {len(received)}')
        # Sequence numbers should advance across frames.
        seqs = [m.frame_number for m in received]
        self.assertEqual(seqs, sorted(seqs), 'frame_number should be monotonic')

    def test_depth_metadata_topic_published(self):
        received = []
        sub = self.node.create_subscription(
            Metadata, '/vxl_camera/depth/metadata',
            lambda m: received.append(m), qos_profile_sensor_data)
        self._spin_until(lambda: len(received) >= 3, timeout_s=5.0)
        sub
        self.assertGreaterEqual(len(received), 3)

    def test_extrinsics_latched(self):
        # Extrinsics is published once with TRANSIENT_LOCAL — late subscribers get it.
        received = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sub = self.node.create_subscription(
            Extrinsics, '/vxl_camera/extrinsics/depth_to_color',
            lambda m: received.append(m), qos)
        self._spin_until(lambda: len(received) > 0, timeout_s=3.0)
        sub
        self.assertGreater(len(received), 0, 'No latched extrinsics received')
        self.assertEqual(len(received[0].rotation), 9)
        self.assertEqual(len(received[0].translation), 3)

    def test_diagnostics_published(self):
        received = []
        sub = self.node.create_subscription(
            DiagnosticArray, '/diagnostics',
            lambda m: received.append(m), 10)
        # diagnostic_updater fires at 1 Hz; wait for ≥2 cycles.
        self._spin_until(lambda: len(received) >= 2, timeout_s=4.0)
        sub
        self.assertGreaterEqual(len(received), 1, 'No /diagnostics messages received')
        # Verify our entry is present and reports streaming.
        all_status = [s for arr in received for s in arr.status]
        vxl_status = [s for s in all_status if 'vxl_camera' in s.name]
        self.assertGreater(len(vxl_status), 0,
            f'No vxl_camera diagnostic status; saw: {[s.name for s in all_status]}')

    def test_color_exposure_param_settable(self):
        # The mock pre-registers VXL_OPTION_EXPOSURE — set should succeed end-to-end.
        from rcl_interfaces.srv import SetParameters
        from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

        client = self.node.create_client(SetParameters, '/vxl_camera/set_parameters')
        self.assertTrue(client.wait_for_service(timeout_sec=5.0))

        req = SetParameters.Request()
        req.parameters = [Parameter(
            name='color.exposure',
            value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=7777))]
        future = client.call_async(req)
        self._spin_until(lambda: future.done(), timeout_s=3.0)
        self.assertTrue(future.done())
        self.assertTrue(future.result().results[0].successful,
            f'set_parameters failed: {future.result().results[0].reason}')

    def test_cold_param_rejected_over_service(self):
        from rcl_interfaces.srv import SetParameters
        from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

        client = self.node.create_client(SetParameters, '/vxl_camera/set_parameters')
        self.assertTrue(client.wait_for_service(timeout_sec=5.0))

        req = SetParameters.Request()
        req.parameters = [Parameter(
            name='color.width',
            value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=320))]
        future = client.call_async(req)
        self._spin_until(lambda: future.done(), timeout_s=3.0)
        self.assertTrue(future.done())
        self.assertFalse(future.result().results[0].successful,
            'cold parameter color.width should have been rejected')
        self.assertIn('cannot be changed at runtime',
            future.result().results[0].reason)
