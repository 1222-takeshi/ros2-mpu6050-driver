# Copyright 2026 Takeshi Miura
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pathlib import Path
import unittest

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from launch_testing.actions import ReadyToTest
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
import rclpy


DEFAULTS = {
    'frame_id': 'imu',
    'publish_rate_hz': 100.0,
    'angular_velocity_bias': [0.0] * 3,
    'linear_acceleration_bias': [0.0] * 3,
    'angular_velocity_covariance': [0.0] * 9,
    'linear_acceleration_covariance': [0.0] * 9,
}
OVERRIDES = {
    'frame_id': 'imu_link',
    'publish_rate_hz': 200.0,
    'angular_velocity_bias': [0.001, -0.002, 0.0005],
    'linear_acceleration_bias': [0.05, -0.03, 0.10],
    'angular_velocity_covariance': [0.0004, 0.0, 0.0, 0.0, 0.0004, 0.0, 0.0, 0.0, 0.0004],
    'linear_acceleration_covariance': [0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.04],
}


def generate_test_description():
    launch_file = Path(__file__).resolve().parents[1] / 'launch' / 'mpu6050_driver.launch.xml'
    cases = []
    for namespace, arguments in (
        ('default_case', {}),
        ('override_case', {name: str(value) for name, value in OVERRIDES.items()}),
    ):
        cases.append(GroupAction([
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(str(launch_file)),
                launch_arguments=arguments.items(),
            ),
        ]))
    return LaunchDescription(cases + [ReadyToTest()])


class TestLaunchParameters(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_launch_parameters')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def assert_parameters(self, namespace, expected):
        client = self.node.create_client(
            GetParameters, f'/{namespace}/imu_driver/get_parameters')
        self.assertTrue(client.wait_for_service(timeout_sec=15.0))
        names = list(expected)
        request = GetParameters.Request()
        request.names = names
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(future.done(), 'Parameter service response timed out')
        response = future.result()
        self.assertIsNotNone(response)
        self.assertEqual(len(response.values), len(names))
        for name, value in zip(names, response.values):
            with self.subTest(parameter=name):
                if isinstance(expected[name], list):
                    self.assertEqual(value.type, ParameterType.PARAMETER_DOUBLE_ARRAY)
                    self.assertEqual(list(value.double_array_value), expected[name])
                elif isinstance(expected[name], str):
                    self.assertEqual(value.type, ParameterType.PARAMETER_STRING)
                    self.assertEqual(value.string_value, expected[name])
                else:
                    self.assertEqual(value.type, ParameterType.PARAMETER_DOUBLE)
                    self.assertEqual(value.double_value, expected[name])

    def test_default_parameter_types_and_values(self):
        self.assert_parameters('default_case', DEFAULTS)

    def test_readme_overrides_reach_node(self):
        self.assert_parameters('override_case', OVERRIDES)
