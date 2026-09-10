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

import ast
import pathlib

PURE_MODULES = ["layout_generator.py", "sdf_templates.py"]
FORBIDDEN = {
    "rclpy", "geometry_msgs", "sensor_msgs", "std_srvs",
    "ros_gz_interfaces", "rosidl_runtime_py", "builtin_interfaces",
}


def _imported_names(source: str) -> set[str]:
    tree = ast.parse(source)
    names: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names.update(a.name.split(".")[0] for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            names.add(node.module.split(".")[0])
    return names


def test_pure_modules_have_no_ros_imports():
    pkg = pathlib.Path(__file__).resolve().parents[1] / "scene_spawner"
    for mod in PURE_MODULES:
        path = pkg / mod
        assert path.exists(), f"{mod} missing"
        offending = _imported_names(path.read_text()) & FORBIDDEN
        assert not offending, f"{mod} imports {offending}"
