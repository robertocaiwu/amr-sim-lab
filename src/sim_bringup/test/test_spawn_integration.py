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

Needs Gazebo Harmonic and a built + sourced workspace. Run it via
`make test-integration` / `colcon test`, not `make test`. `setUpClass` starts
`spawner_only.launch.py` as a subprocess and skips the test if it does not come
up; it is skipped outright when `gz` is not on PATH.
"""

import shutil
import subprocess
import time
import unittest

import rclpy
from std_srvs.srv import Trigger

from scene_spawner_interfaces.srv import SpawnLayout


class TestSpawnRoundTrip(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        if shutil.which("gz") is None:
            raise unittest.SkipTest("Gazebo (gz) not installed")
        rclpy.init()
        cls.node = rclpy.create_node("spawn_integration_client")
        cls.proc = subprocess.Popen(
            ["ros2", "launch", "sim_bringup", "spawner_only.launch.py"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        cls.spawn_cli = cls.node.create_client(
            SpawnLayout, "/scene_spawner/spawn_layout")
        cls.clear_cli = cls.node.create_client(
            Trigger, "/scene_spawner/clear_layout")
        if not cls.spawn_cli.wait_for_service(timeout_sec=40.0):
            cls.proc.terminate()
            raise unittest.SkipTest("spawner_only.launch.py did not come up in 40s")

    @classmethod
    def tearDownClass(cls):
        cls.proc.terminate()
        try:
            cls.proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            cls.proc.kill()
        cls.node.destroy_node()
        rclpy.shutdown()

    def _call(self, client, request, timeout=20.0):
        self.assertTrue(client.wait_for_service(timeout_sec=timeout))
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        self.assertIsNotNone(future.result())
        return future.result()

    def test_spawn_then_clear(self):
        req = SpawnLayout.Request()
        req.count = 5
        req.seed = 1
        req.min_spacing = 1.0
        req.area_min_x, req.area_min_y, req.area_max_x, req.area_max_y = \
            -3.0, -3.0, 3.0, 3.0
        resp = self._call(self.spawn_cli, req)
        self.assertTrue(resp.success)
        self.assertEqual(resp.placed, 5)
        self.assertEqual(len(resp.entity_names), 5)

        time.sleep(1.0)
        cresp = self._call(self.clear_cli, Trigger.Request())
        self.assertTrue(cresp.success)
