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

"""ROS 2 node: procedurally spawn / clear scene-prop layouts in Gazebo."""

from __future__ import annotations

import shutil
import subprocess
import threading
import time

import rclpy
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from scene_spawner.layout_generator import InvalidArea, LayoutParams, generate_layout
from scene_spawner.sdf_templates import include_spawn_sdf
from scene_spawner_interfaces.srv import SpawnLayout

# The create/remove entry points are Gazebo Transport services, not ROS
# services. `ros_gz_bridge`'s parameter_bridge only bridges topics (its
# config parser rejects service entries), so this node talks to Gazebo
# directly through the `gz service` CLI, which shares the transport bus
# with the simulator when both run in the same environment.
_GZ_REQ_TIMEOUT_MS = "3000"


class SpawnerNode(Node):
    """Owns the current layout and drives Gazebo's create/remove services."""

    def __init__(self) -> None:
        super().__init__("scene_spawner")
        self._cb = ReentrantCallbackGroup()

        self.declare_parameter("world_name", "warehouse")
        self.declare_parameter("default_count", 8)
        self.declare_parameter("default_area", [-4.0, -4.0, 4.0, 4.0])
        self.declare_parameter("default_min_spacing", 1.0)
        self.declare_parameter("default_model", "cart")
        self.declare_parameter("cart_footprint_radius", 0.5)
        self.declare_parameter("spawn_on_startup", True)
        self.declare_parameter("startup_seed", 0)
        self.declare_parameter("create_timeout_sec", 10.0)

        world = self.get_parameter("world_name").value
        self._create_name = f"/world/{world}/create"
        self._remove_name = f"/world/{world}/remove"

        self._lock = threading.Lock()
        self._spawned: list[str] = []

        self.create_service(
            SpawnLayout, "~/spawn_layout", self._on_spawn_layout,
            callback_group=self._cb)
        self.create_service(
            Trigger, "~/clear_layout", self._on_clear_layout,
            callback_group=self._cb)

        self._startup_done = False
        if bool(self.get_parameter("spawn_on_startup").value):
            # Its own mutually-exclusive group: a repeating timer whose
            # callback can take several seconds must not overlap itself
            # (that caused clear+respawn churn on startup).
            self._startup_timer = self.create_timer(
                2.0, self._startup_spawn_once,
                callback_group=MutuallyExclusiveCallbackGroup())

    # -- helpers ---------------------------------------------------------

    def _gz_available(self) -> bool:
        """True once the Gazebo create service is advertised on the bus."""
        if shutil.which("gz") is None:
            self.get_logger().error("`gz` CLI not found on PATH")
            return False
        timeout = float(self.get_parameter("create_timeout_sec").value)
        deadline = time.monotonic() + max(timeout, 0.5)
        while time.monotonic() < deadline:
            try:
                out = subprocess.run(
                    ["gz", "service", "-l"],
                    capture_output=True, text=True, timeout=5.0)
            except (subprocess.TimeoutExpired, OSError):
                return False
            if self._create_name in out.stdout:
                return True
            time.sleep(0.5)
        return False

    @staticmethod
    def _gz_string(value: str) -> str:
        """Escape a value for a Gazebo Transport text-format string field."""
        return value.replace("\\", "\\\\").replace('"', '\\"')

    def _gz_request(self, service: str, req_type: str, req_body: str) -> bool:
        cmd = ["gz", "service", "-s", service,
               "--reqtype", req_type, "--reptype", "gz.msgs.Boolean",
               "--timeout", _GZ_REQ_TIMEOUT_MS, "--req", req_body]
        try:
            out = subprocess.run(
                cmd, capture_output=True, text=True, timeout=10.0)
        except (subprocess.TimeoutExpired, OSError) as exc:
            self.get_logger().warning(f"gz service {service} failed: {exc}")
            return False
        if "data: true" in out.stdout:
            return True
        detail = (out.stdout or out.stderr).strip().replace("\n", " ")
        self.get_logger().warning(
            f"gz service {service} did not confirm: {detail[:200]}")
        return False

    def _params_from_request(self, req: SpawnLayout.Request) -> LayoutParams:
        count = req.count if req.count > 0 else int(
            self.get_parameter("default_count").value)
        spacing = req.min_spacing if req.min_spacing > 0.0 else float(
            self.get_parameter("default_min_spacing").value)
        if req.area_max_x > req.area_min_x and req.area_max_y > req.area_min_y:
            area = (req.area_min_x, req.area_min_y, req.area_max_x, req.area_max_y)
        else:
            area = tuple(self.get_parameter("default_area").value)  # type: ignore[assignment]
        seed = None if req.seed < 0 else req.seed
        return LayoutParams(
            count=count,
            area=area,
            min_spacing=spacing,
            footprint_radius=float(self.get_parameter("cart_footprint_radius").value),
            yaw_jitter=True,
            seed=seed,
        )

    def _spawn_one(self, name: str, x: float, y: float, yaw: float, uri: str) -> bool:
        sdf = include_spawn_sdf(name, x, y, yaw, model_uri=uri)
        return self._gz_request(
            self._create_name, "gz.msgs.EntityFactory",
            f'sdf: "{self._gz_string(sdf)}"')

    def _remove_one(self, name: str) -> bool:
        return self._gz_request(
            self._remove_name, "gz.msgs.Entity",
            f'name: "{self._gz_string(name)}" type: MODEL')

    def _clear_locked(self) -> int:
        removed = 0
        for name in list(self._spawned):
            if self._remove_one(name):
                removed += 1
            self._spawned.remove(name)
        return removed

    # -- service callbacks ---------------------------------------------

    def _on_spawn_layout(self, req: SpawnLayout.Request,
                         resp: SpawnLayout.Response) -> SpawnLayout.Response:
        model = req.model_name or str(self.get_parameter("default_model").value)
        uri = f"model://{model}"
        params = self._params_from_request(req)

        try:
            placements = generate_layout(params, name_prefix=model)
        except InvalidArea:
            resp.success = False
            resp.message = "invalid area bounds"
            return resp

        if not self._gz_available():
            resp.success = False
            resp.message = f"Gazebo create service {self._create_name} unavailable"
            return resp

        with self._lock:
            self._clear_locked()
            names: list[str] = []
            for p in placements:
                if self._spawn_one(p.name, p.x, p.y, p.yaw, uri):
                    self._spawned.append(p.name)
                    names.append(p.name)

        resp.success = True
        resp.requested = params.count
        resp.placed = len(names)
        resp.entity_names = names
        if resp.placed < resp.requested:
            resp.message = (
                f"placed {resp.placed}/{resp.requested} "
                "(area saturated or spawn rejected)")
        else:
            resp.message = f"placed {resp.placed} '{model}' entities"
        return resp

    def _on_clear_layout(self, _req: Trigger.Request,
                         resp: Trigger.Response) -> Trigger.Response:
        with self._lock:
            if not self._spawned:
                resp.success = True
                resp.message = "nothing to clear"
                return resp
            removed = self._clear_locked()
        resp.success = True
        resp.message = f"removed {removed} entities"
        return resp

    def _startup_spawn_once(self) -> None:
        if self._startup_done:
            return
        req = SpawnLayout.Request()
        req.count = 0            # -> default_count
        req.min_spacing = 0.0    # -> default_min_spacing
        req.area_min_x = req.area_min_y = req.area_max_x = req.area_max_y = 0.0
        req.seed = int(self.get_parameter("startup_seed").value)
        req.model_name = ""
        resp = self._on_spawn_layout(req, SpawnLayout.Response())
        if resp.success:
            self._startup_done = True
            self._startup_timer.cancel()
            self.get_logger().info(f"startup spawn: {resp.message}")
        else:
            self.get_logger().warning(
                f"startup spawn not ready yet ({resp.message}); retrying")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SpawnerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
