# Copyright 2026 amr-sim-lab contributors
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

"""Integration test: spawn_layout places N carts, clear_layout removes them.

Requires Gazebo Harmonic on PATH. Skipped automatically if `gz` is absent.
"""

import shutil
import time
import unittest

import pytest
import rclpy
from std_srvs.srv import Trigger

from scene_spawner_interfaces.srv import SpawnLayout

GZ = shutil.which("gz")


@pytest.mark.skipif(GZ is None, reason="Gazebo (gz) not installed")
class TestSpawnRoundTrip(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("integration_test_client")
        # Assumes `ros2 launch sim_bringup spawner_only.launch.py` is already
        # running (the Makefile target / CI job starts it); or use
        # launch_testing to own the process. See launch_testing variant below.

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _call(self, client, request, timeout=20.0):
        self.assertTrue(client.wait_for_service(timeout_sec=timeout))
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        self.assertIsNotNone(future.result())
        return future.result()

    def test_spawn_then_clear(self):
        spawn = self.node.create_client(SpawnLayout, "/scene_spawner/spawn_layout")
        clear = self.node.create_client(Trigger, "/scene_spawner/clear_layout")

        req = SpawnLayout.Request()
        req.count = 5
        req.seed = 1
        req.min_spacing = 1.0
        req.area_min_x, req.area_min_y, req.area_max_x, req.area_max_y = -3.0, -3.0, 3.0, 3.0
        resp = self._call(spawn, req)
        self.assertTrue(resp.success)
        self.assertEqual(resp.placed, 5)
        self.assertEqual(len(resp.entity_names), 5)

        time.sleep(1.0)
        cresp = self._call(clear, Trigger.Request())
        self.assertTrue(cresp.success)


# launch_testing variant (preferred if `ros_gz_sim` launch integration is
# confirmed in Task 8): wrap the above with `launch_testing.main`, having
# `generate_test_description()` `IncludeLaunchDescription(spawner_only.launch.py)`
# + `ReadyToTest()`, so the test owns Gazebo's lifecycle. Use whichever the ROS
# box shows to be reliable; document the choice in a module docstring.
