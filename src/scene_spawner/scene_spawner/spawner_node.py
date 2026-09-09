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

import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity
from std_srvs.srv import Trigger

from scene_spawner.layout_generator import InvalidArea, LayoutParams, generate_layout
from scene_spawner.sdf_templates import include_spawn_sdf
from scene_spawner_interfaces.srv import SpawnLayout


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

        self._create_cli = self.create_client(
            SpawnEntity, self._create_name, callback_group=self._cb)
        self._remove_cli = self.create_client(
            DeleteEntity, self._remove_name, callback_group=self._cb)

        self._lock = threading.Lock()
        self._spawned: list[str] = []

        self.create_service(
            SpawnLayout, "~/spawn_layout", self._on_spawn_layout,
            callback_group=self._cb)
        self.create_service(
            Trigger, "~/clear_layout", self._on_clear_layout,
            callback_group=self._cb)

        if bool(self.get_parameter("spawn_on_startup").value):
            self._startup_timer = self.create_timer(
                1.0, self._startup_spawn_once, callback_group=self._cb)

    # -- helpers ---------------------------------------------------------

    def _wait_for_create(self) -> bool:
        timeout = float(self.get_parameter("create_timeout_sec").value)
        return self._create_cli.wait_for_service(timeout_sec=timeout)

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

    def _call_sync(self, client, request, timeout_sec: float = 5.0):
        """Call a service and block for the response without spinning.

        Safe from inside a callback: this node runs on a MultiThreadedExecutor
        with a ReentrantCallbackGroup, so the response is delivered by another
        executor thread while this one waits.
        """
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout_sec):
            future.cancel()
            return None
        return future.result()

    def _spawn_one(self, name: str, x: float, y: float, yaw: float, uri: str) -> bool:
        req = SpawnEntity.Request()
        req.entity_factory.name = name
        req.entity_factory.sdf = include_spawn_sdf(name, x, y, yaw, model_uri=uri)
        req.entity_factory.allow_renaming = False
        result = self._call_sync(self._create_cli, req)
        return bool(result and result.success)

    def _remove_one(self, name: str) -> bool:
        req = DeleteEntity.Request()
        req.entity.name = name
        req.entity.type = Entity.MODEL
        result = self._call_sync(self._remove_cli, req)
        return bool(result and result.success)

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

        if not self._wait_for_create():
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
        self._startup_timer.cancel()
        req = SpawnLayout.Request()
        req.count = 0            # -> default_count
        req.min_spacing = 0.0    # -> default_min_spacing
        req.area_min_x = req.area_min_y = req.area_max_x = req.area_max_y = 0.0
        req.seed = int(self.get_parameter("startup_seed").value)
        req.model_name = ""
        resp = SpawnLayout.Response()
        resp = self._on_spawn_layout(req, resp)
        self.get_logger().info(f"startup spawn: {resp.message}")


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
