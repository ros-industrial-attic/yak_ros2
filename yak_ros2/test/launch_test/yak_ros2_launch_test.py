import os
import unittest

from ament_index_python import get_package_share_directory, get_package_prefix
import shutil
import tempfile
import time

import rclpy
from rclpy.duration import Duration
from rclpy.time import Time

import launch
import launch.actions
from launch.launch_description_sources import PythonLaunchDescriptionSource

import launch_testing
import launch_testing.actions

from std_srvs.srv import Trigger

import pytest

@pytest.mark.launch_test
def generate_test_description():
    demo_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('yak_ros2'), '/launch/demo.launch.py']))

    return launch.LaunchDescription([
        demo_launch,
        launch_testing.actions.ReadyToTest(),
    ])


class TestSetTransforms(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

    def setUp(self):
        # Create a ROS node for tests
        self.node = rclpy.create_node('test_mutable_tf_pub')
        self.gen_mesh_client = self.node.create_client(Trigger, 'generate_mesh_service')

    def tearDown(self):
        self.node.destroy_node()

    def test_tf_lookup(self, proc_output):
        # spin for a few cycles while the TSDF node initializes and fuses some images
        time.sleep(5.0)

        req_msg = Trigger.Request()
        future = self.gen_mesh_client.call_async(req_msg)

        while rclpy.ok():
            rclpy.spin_once(self.node)
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    self.node.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    self.node.get_logger().info(response.success)
                break

        assert response.success is True


@launch_testing.post_shutdown_test()
class TestYamlAfterShutdown(unittest.TestCase):
    # TODO: test if the file is in the expected state after tests finish and nodes shut down
    pass
