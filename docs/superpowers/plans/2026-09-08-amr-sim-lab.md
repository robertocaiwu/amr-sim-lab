# amr-sim-lab Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a self-contained ROS 2 Humble + Gazebo Harmonic project that procedurally spawns randomized, reproducible cart layouts into a warehouse world and lets you drive a lidar-equipped AMR through it with an obstacle safety-stop.

**Architecture:** Four colcon packages. A pure-Python core (`layout_generator`, `sdf_templates`, `gate`) with zero ROS/Gazebo imports holds all the testable logic. Thin ROS 2 nodes wrap that core: `scene_spawner` exposes a `SpawnLayout` service and drives Gazebo's spawn/remove services; `safety_stop` gates `/cmd_vel_raw` → `/cmd_vel` using `/scan`. `sim_bringup` owns all launch files and the `ros_gz` bridge config. The scene definition (`layout.yaml`), the cart geometry, and the layout algorithm are kept stack-agnostic so a later Isaac Sim + Replicator port reuses them directly.

**Tech Stack:** ROS 2 Humble, Python 3.10, Gazebo Harmonic (`gz-sim` 8), SDF 1.10, `ros_gz` (`ros-humble-ros-gzharmonic`), `pytest`, `launch_testing`, Docker.

**Spec:** `docs/superpowers/specs/2026-09-08-amr-sim-lab-design.md`

## Global Constraints

- **ROS distro:** ROS 2 Humble. **Python:** 3.10 (system Python on Ubuntu 22.04). **Gazebo:** Harmonic / `gz-sim` 8, installed via `ros-humble-ros-gzharmonic` from the OSRF apt repo — never assume Fortress; never `apt install gazebo` or `ros-gz` unqualified.
- **SDF version:** every SDF/world file declares `<sdf version="1.10">`.
- **License header:** every source file created by this plan begins with the Apache-2.0 header block (exact text in Task 1, Step 2). Python/`.launch.py`: `#` comments. XML/SDF: `<!-- ... -->` after the `<?xml?>` line. YAML/CMake: `#` comments.
- **Purity rule:** `scene_spawner/scene_spawner/layout_generator.py`, `scene_spawner/scene_spawner/sdf_templates.py`, and `safety_stop/safety_stop/gate.py` MUST NOT import `rclpy`, `geometry_msgs`, `sensor_msgs`, `std_srvs`, `ros_gz_interfaces`, or any other ROS/Gazebo package. Enforced by a test in each package.
- **RNG:** always use a local `random.Random(seed)` instance. Never call the `random` module's module-level functions.
- **Repo name:** `amr-sim-lab`. **License:** Apache-2.0. No references anywhere (code, comments, docs, commit messages) to any employer, recruiter, job posting, or job-application process — this is an independent project.
- **Commits:** one commit per completed task, message prefixed `feat:`, `test:`, `chore:`, or `docs:`. End every commit message with the two trailer lines:
  ```
  Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01Hmj5W4Ci2Nd8JvT6M9VMcG
  ```
- **Build/test environment:** Tasks 1–4 (pure Python + interface file) are fully verifiable in any environment with Python 3.10 + `pytest`. Tasks 5–14 need a machine with ROS 2 Humble + Gazebo Harmonic for full verification; each such task lists a **local static check** (runs anywhere) and a **machine check** (runs on the ROS box). Do not mark a machine check `[x]` until it has actually been run on the ROS box.
- **Pure-test invocation:** from the package directory, `python3 -m pytest test/ -v`. `pytest` adds the package dir to `sys.path` because `test/` has no `__init__.py`, so `import scene_spawner.layout_generator` resolves without an install.

---

## File Structure

```
/workspace/simulation/                      # repo root, git already initialised
├── LICENSE                                 # Apache-2.0 full text                     [Task 1]
├── NOTICE                                  # attribution (Dolly)                       [Task 1]
├── README.md                               # overview, quickstart, Phase 2 roadmap    [Task 13]
├── .gitignore                              # exists (build/ install/ log/ __pycache__)
├── Makefile                                # build / test / test-integration / run    [Task 13]
├── scripts/demo.sh                         # build + launch + sample service call     [Task 13]
├── .devcontainer/devcontainer.json         # VS Code devcontainer -> docker/Dockerfile [Task 12]
├── docker/
│   ├── Dockerfile                          # ros:humble + Harmonic + deps             [Task 12]
│   └── entrypoint.sh                        # source ROS + workspace overlay           [Task 12]
├── docs/
│   ├── superpowers/specs/2026-09-08-amr-sim-lab-design.md   # the spec (exists)
│   ├── superpowers/plans/2026-09-08-amr-sim-lab.md          # this file
│   ├── gazebo-vs-isaac.md                  # Phase 2 comparison notes (seed)          [Task 13]
│   └── smoke-test.md                       # manual teleop checklist                   [Task 13]
├── worlds/warehouse.sdf                    # Harmonic world: floor, walls, lights      [Task 9]
├── models/
│   ├── cart/                                # primitive-geometry cart model            [Task 10]
│   │   ├── model.config
│   │   └── model.sdf
│   └── dolly/                               # git submodule: github.com/chapulina/dolly [Task 11]
├── config/layout.yaml                      # symlink -> src/sim_bringup/config/layout.yaml [Task 7]
└── src/
    ├── scene_spawner_interfaces/            # SpawnLayout.srv                           [Task 4]
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── srv/SpawnLayout.srv
    ├── scene_spawner/
    │   ├── package.xml
    │   ├── setup.py
    │   ├── setup.cfg
    │   ├── resource/scene_spawner
    │   ├── scene_spawner/
    │   │   ├── __init__.py
    │   │   ├── layout_generator.py          # pure                                     [Task 2]
    │   │   ├── sdf_templates.py             # pure                                     [Task 3]
    │   │   └── spawner_node.py              # ROS node                                 [Task 5]
    │   └── test/
    │       ├── test_layout_generator.py                                               [Task 2]
    │       ├── test_sdf_templates.py                                                   [Task 3]
    │       └── test_no_ros_imports.py                                                  [Task 2]
    ├── safety_stop/
    │   ├── package.xml
    │   ├── setup.py
    │   ├── setup.cfg
    │   ├── resource/safety_stop
    │   ├── safety_stop/
    │   │   ├── __init__.py
    │   │   ├── gate.py                      # pure                                     [Task 6]
    │   │   └── safety_stop_node.py          # ROS node                                 [Task 6]
    │   └── test/
    │       ├── test_gate.py                                                            [Task 6]
    │       └── test_no_ros_imports.py                                                  [Task 6]
    └── sim_bringup/
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── resource/sim_bringup
        ├── config/
        │   ├── bridge.yaml                                                             [Task 7]
        │   └── layout.yaml                  # canonical copy                           [Task 7]
        ├── launch/
        │   ├── spawner_only.launch.py                                                  [Task 8]
        │   └── sim.launch.py                                                           [Task 8]
        └── test/test_spawn_integration.py   # launch_testing                           [Task 14]
```

---

## Task 1: Repo skeleton — license, notice, package directories

**Files:**
- Create: `LICENSE`, `NOTICE`
- Create: `src/scene_spawner/package.xml`, `src/scene_spawner/setup.py`, `src/scene_spawner/setup.cfg`, `src/scene_spawner/resource/scene_spawner`, `src/scene_spawner/scene_spawner/__init__.py`
- Create: `src/safety_stop/package.xml`, `src/safety_stop/setup.py`, `src/safety_stop/setup.cfg`, `src/safety_stop/resource/safety_stop`, `src/safety_stop/safety_stop/__init__.py`
- Create: `src/sim_bringup/package.xml`, `src/sim_bringup/setup.py`, `src/sim_bringup/setup.cfg`, `src/sim_bringup/resource/sim_bringup`

**Interfaces:**
- Consumes: nothing.
- Produces: three ament_python package skeletons named `scene_spawner`, `safety_stop`, `sim_bringup`. Console entry points added in later tasks.

- [ ] **Step 1: Create the Apache-2.0 LICENSE file**

Write the full Apache License 2.0 text to `LICENSE`. Fetch the canonical text from https://www.apache.org/licenses/LICENSE-2.0.txt (it is ~11 KB). The copyright line in the appendix stays as the generic `Copyright [yyyy] [name of copyright owner]` — do not fill in a name.

- [ ] **Step 2: Record the license header block**

Create `NOTICE` with this content:

```
amr-sim-lab
Copyright 2026

This product includes software developed as an independent project.

Licensed under the Apache License, Version 2.0.

--------------------------------------------------------------------------------
Third-party components:

- Dolly (models/dolly/), Copyright (c) Louise Poubel and contributors,
  licensed under the Apache License, Version 2.0.
  Source: https://github.com/chapulina/dolly
```

The per-file header block that every source file created by this plan must start with (this exact text — adjust only the comment syntax):

```
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
```

- [ ] **Step 3: Create the `scene_spawner` package skeleton**

`src/scene_spawner/package.xml` (start with the header block as an XML comment after the `<?xml?>` line):

```xml
<?xml version="1.0"?>
<!--
Copyright 2026 amr-sim-lab contributors
Licensed under the Apache License, Version 2.0. See LICENSE.
-->
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>scene_spawner</name>
  <version>0.1.0</version>
  <description>Procedural, reproducible spawning of scene props into a Gazebo world.</description>
  <maintainer email="you@example.com">amr-sim-lab contributors</maintainer>
  <license>Apache-2.0</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_srvs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>ros_gz_interfaces</exec_depend>
  <exec_depend>scene_spawner_interfaces</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

`src/scene_spawner/setup.py`:

```python
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

from setuptools import find_packages, setup

package_name = 'scene_spawner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amr-sim-lab contributors',
    maintainer_email='you@example.com',
    description='Procedural, reproducible spawning of scene props into a Gazebo world.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'spawner_node = scene_spawner.spawner_node:main',
        ],
    },
)
```

`src/scene_spawner/setup.cfg`:

```
[develop]
script_dir=$base/lib/scene_spawner
[install]
install_scripts=$base/lib/scene_spawner
```

`src/scene_spawner/resource/scene_spawner`: empty file.

`src/scene_spawner/scene_spawner/__init__.py`: header block only.

- [ ] **Step 4: Create the `safety_stop` and `sim_bringup` skeletons**

Same structure. For `safety_stop/package.xml` the deps are `rclpy`, `geometry_msgs`, `sensor_msgs`; entry point `safety_stop_node = safety_stop.safety_stop_node:main`. For `sim_bringup/package.xml` use `<build_type>ament_python</build_type>`, deps `rclpy`, `ros_gz_sim`, `ros_gz_bridge`, `teleop_twist_keyboard`, `scene_spawner`, `safety_stop`; no console scripts (launch-only). `sim_bringup/setup.py` `data_files` must also install `launch/` and `config/`:

```python
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/spawner_only.launch.py', 'launch/sim.launch.py']),
        ('share/' + package_name + '/config', [
            'config/bridge.yaml', 'config/layout.yaml']),
    ],
```

(The `launch/` and `config/` files themselves are created in Tasks 7–8; listing them here is fine — they must exist before `colcon build`, which first runs in Task 5.)

- [ ] **Step 5: Verify structure**

Run: `find /workspace/simulation/src -type f | sort`
Expected: all skeleton files listed, three `package.xml`, three `setup.py`.
Run: `python3 -c "import xml.dom.minidom, pathlib; [xml.dom.minidom.parseString(p.read_text()) for p in pathlib.Path('/workspace/simulation/src').rglob('package.xml')]; print('all package.xml well-formed')"`
Expected: `all package.xml well-formed`

- [ ] **Step 6: Commit**

```bash
cd /workspace/simulation
git add LICENSE NOTICE src/
git commit -m "chore: scaffold colcon packages and license

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Hmj5W4Ci2Nd8JvT6M9VMcG"
```

---

## Task 2: `layout_generator` — pure procedural placement

**Files:**
- Create: `src/scene_spawner/scene_spawner/layout_generator.py`
- Test: `src/scene_spawner/test/test_layout_generator.py`
- Test: `src/scene_spawner/test/test_no_ros_imports.py`

**Interfaces:**
- Consumes: nothing.
- Produces:
  - `LayoutParams(count: int, area: tuple[float,float,float,float], min_spacing: float, footprint_radius: float, yaw_jitter: bool = True, seed: int | None = None)` — frozen dataclass. `area` is `(min_x, min_y, max_x, max_y)`.
  - `Placement(name: str, x: float, y: float, yaw: float)` — frozen dataclass.
  - `InvalidArea(ValueError)` — exception.
  - `generate_layout(params: LayoutParams, name_prefix: str = "cart") -> list[Placement]` — deterministic for a non-`None` seed; raises `InvalidArea` on degenerate bounds; returns `[]` for `count <= 0`; returns a partial list (never raises) when the area saturates.
  - `MAX_ATTEMPTS_PER_ITEM: int = 100` — module constant.

- [ ] **Step 1: Write the failing tests**

`src/scene_spawner/test/test_layout_generator.py`:

```python
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

import math

import pytest

from scene_spawner.layout_generator import (
    InvalidArea,
    LayoutParams,
    Placement,
    generate_layout,
)

AREA = (-4.0, -4.0, 4.0, 4.0)


def _params(**kw):
    base = dict(count=6, area=AREA, min_spacing=1.0, footprint_radius=0.5,
               yaw_jitter=True, seed=1)
    base.update(kw)
    return LayoutParams(**base)


def test_returns_requested_count_when_space_allows():
    result = generate_layout(_params(count=6))
    assert len(result) == 6
    assert all(isinstance(p, Placement) for p in result)


def test_deterministic_for_same_seed():
    a = generate_layout(_params(seed=42))
    b = generate_layout(_params(seed=42))
    assert a == b


def test_different_seed_gives_different_layout():
    a = generate_layout(_params(seed=1, count=8))
    b = generate_layout(_params(seed=2, count=8))
    assert a != b


def test_all_placements_within_area():
    for p in generate_layout(_params(count=10)):
        assert AREA[0] <= p.x <= AREA[2]
        assert AREA[1] <= p.y <= AREA[3]


def test_respects_minimum_spacing():
    params = _params(count=10, min_spacing=1.5)
    clearance = max(params.min_spacing, 2 * params.footprint_radius)
    pts = generate_layout(params)
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            d = math.hypot(pts[i].x - pts[j].x, pts[i].y - pts[j].y)
            assert d >= clearance - 1e-9


def test_over_capacity_returns_partial_without_raising():
    # A 1x1 m area cannot hold 50 carts 1 m apart.
    result = generate_layout(_params(count=50, area=(0.0, 0.0, 1.0, 1.0)))
    assert len(result) < 50


def test_zero_count_returns_empty():
    assert generate_layout(_params(count=0)) == []


def test_negative_count_returns_empty():
    assert generate_layout(_params(count=-3)) == []


def test_degenerate_area_raises():
    with pytest.raises(InvalidArea):
        generate_layout(_params(area=(1.0, 0.0, 1.0, 5.0)))


def test_yaw_jitter_disabled_gives_zero_yaw():
    for p in generate_layout(_params(yaw_jitter=False)):
        assert p.yaw == 0.0


def test_yaw_jitter_enabled_varies():
    yaws = {round(p.yaw, 3) for p in generate_layout(_params(count=8, yaw_jitter=True))}
    assert len(yaws) > 1


def test_name_prefix_and_zero_padding():
    result = generate_layout(_params(count=3), name_prefix="box")
    assert [p.name for p in result] == ["box_00", "box_01", "box_02"]
```

`src/scene_spawner/test/test_no_ros_imports.py`:

```python
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
        if not path.exists():
            continue
        offending = _imported_names(path.read_text()) & FORBIDDEN
        assert not offending, f"{mod} imports {offending}"
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd /workspace/simulation/src/scene_spawner && python3 -m pytest test/ -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'scene_spawner.layout_generator'` (the no-ros-imports test passes vacuously since the file does not exist yet).

- [ ] **Step 3: Implement `layout_generator.py`**

```python
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

"""Deterministic, dependency-free procedural placement of scene props.

Pure module: no ROS, no Gazebo, no I/O. Given bounds, a count, a minimum
spacing and a seed, produce a reproducible list of non-overlapping poses.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

MAX_ATTEMPTS_PER_ITEM = 100


class InvalidArea(ValueError):
    """Raised when area bounds are degenerate (min >= max on either axis)."""


@dataclass(frozen=True)
class LayoutParams:
    """Inputs to :func:`generate_layout`.

    ``area`` is ``(min_x, min_y, max_x, max_y)`` in world-frame metres.
    ``seed`` of ``None`` means non-deterministic (time-seeded).
    """

    count: int
    area: tuple[float, float, float, float]
    min_spacing: float
    footprint_radius: float
    yaw_jitter: bool = True
    seed: int | None = None


@dataclass(frozen=True)
class Placement:
    """A single prop pose. ``yaw`` is in radians."""

    name: str
    x: float
    y: float
    yaw: float


def generate_layout(params: LayoutParams, name_prefix: str = "cart") -> list[Placement]:
    """Rejection-sample non-overlapping prop poses inside ``params.area``.

    Deterministic for a given non-``None`` seed. Places as many items as
    fit; if an item cannot be placed after ``MAX_ATTEMPTS_PER_ITEM`` tries,
    returns the partial list built so far (never raises for saturation).
    """
    min_x, min_y, max_x, max_y = params.area
    if min_x >= max_x or min_y >= max_y:
        raise InvalidArea(f"degenerate area bounds: {params.area}")
    if params.count <= 0:
        return []

    rng = random.Random(params.seed)
    clearance = max(params.min_spacing, 2.0 * params.footprint_radius)
    placements: list[Placement] = []

    for i in range(params.count):
        placed = False
        for _ in range(MAX_ATTEMPTS_PER_ITEM):
            x = rng.uniform(min_x, max_x)
            y = rng.uniform(min_y, max_y)
            if all(math.hypot(x - p.x, y - p.y) >= clearance for p in placements):
                yaw = rng.uniform(-math.pi, math.pi) if params.yaw_jitter else 0.0
                placements.append(Placement(f"{name_prefix}_{i:02d}", x, y, yaw))
                placed = True
                break
        if not placed:
            break

    return placements
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd /workspace/simulation/src/scene_spawner && python3 -m pytest test/ -v`
Expected: PASS — all tests in `test_layout_generator.py` and `test_no_ros_imports.py`.

- [ ] **Step 5: Commit**

```bash
cd /workspace/simulation
git add src/scene_spawner/scene_spawner/layout_generator.py src/scene_spawner/test/
git commit -m "feat: add deterministic procedural layout generator

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Hmj5W4Ci2Nd8JvT6M9VMcG"
```

---

## Task 3: `sdf_templates` — pure SDF snippet builder

**Files:**
- Create: `src/scene_spawner/scene_spawner/sdf_templates.py`
- Test: `src/scene_spawner/test/test_sdf_templates.py`

**Interfaces:**
- Consumes: nothing.
- Produces:
  - `include_spawn_sdf(name: str, x: float, y: float, yaw: float, *, z: float = 0.0, model_uri: str = "model://cart") -> str` — returns a complete SDF 1.10 document string with a single top-level `<include>` carrying `<name>`, `<uri>`, `<pose>`. Coordinates formatted to 4 decimals; `name` and `model_uri` XML-escaped.

- [ ] **Step 1: Write the failing test**

`src/scene_spawner/test/test_sdf_templates.py`:

```python
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

import xml.etree.ElementTree as ET

from scene_spawner.sdf_templates import include_spawn_sdf


def test_output_is_well_formed_xml():
    ET.fromstring(include_spawn_sdf("cart_00", 1.0, 2.0, 0.5))


def test_include_has_name_uri_pose():
    root = ET.fromstring(include_spawn_sdf("cart_01", 1.5, -2.5, 0.7854,
                                           model_uri="model://cart"))
    inc = root.find("include")
    assert inc is not None
    assert inc.findtext("name") == "cart_01"
    assert inc.findtext("uri") == "model://cart"
    assert inc.findtext("pose") == "1.5000 -2.5000 0.0000 0 0 0.7854"


def test_sdf_version_is_1_10():
    root = ET.fromstring(include_spawn_sdf("cart_00", 0.0, 0.0, 0.0))
    assert root.tag == "sdf"
    assert root.attrib["version"] == "1.10"


def test_special_characters_in_name_are_escaped():
    out = include_spawn_sdf('cart & "x"', 0.0, 0.0, 0.0)
    assert "&amp;" in out
    ET.fromstring(out)  # still parses
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /workspace/simulation/src/scene_spawner && python3 -m pytest test/test_sdf_templates.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'scene_spawner.sdf_templates'`

- [ ] **Step 3: Implement `sdf_templates.py`**

```python
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

"""Pure helpers that build Gazebo SDF snippets for spawning scene props."""

from __future__ import annotations

from xml.sax.saxutils import escape


def include_spawn_sdf(name: str, x: float, y: float, yaw: float, *,
                      z: float = 0.0, model_uri: str = "model://cart") -> str:
    """Return a complete SDF 1.10 document that includes an installed model.

    The string is suitable as the ``sdf`` field of a Gazebo
    ``/world/<world>/create`` request (``ros_gz_interfaces/EntityFactory``).
    """
    safe_name = escape(name, {'"': "&quot;"})
    safe_uri = escape(model_uri, {'"': "&quot;"})
    pose = f"{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.4f}"
    return (
        '<?xml version="1.0" ?>'
        '<sdf version="1.10">'
        "<include>"
        f"<name>{safe_name}</name>"
        f"<uri>{safe_uri}</uri>"
        f"<pose>{pose}</pose>"
        "</include>"
        "</sdf>"
    )
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /workspace/simulation/src/scene_spawner && python3 -m pytest test/ -v`
Expected: PASS — all `scene_spawner` tests (layout + sdf + no-ros-imports).

- [ ] **Step 5: Commit**

```bash
cd /workspace/simulation
git add src/scene_spawner/scene_spawner/sdf_templates.py src/scene_spawner/test/test_sdf_templates.py
git commit -m "feat: add SDF include-spawn template builder

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Hmj5W4Ci2Nd8JvT6M9VMcG"
```

---

## Task 4: `scene_spawner_interfaces` — the `SpawnLayout` service

**Files:**
- Create: `src/scene_spawner_interfaces/package.xml`
- Create: `src/scene_spawner_interfaces/CMakeLists.txt`
- Create: `src/scene_spawner_interfaces/srv/SpawnLayout.srv`

**Interfaces:**
- Consumes: nothing.
- Produces: `scene_spawner_interfaces/srv/SpawnLayout` with the request/response fields below. Consumed by Task 5.

- [ ] **Step 1: Create `srv/SpawnLayout.srv`**

```
# Request
int32   count            # carts to place; 0 => node default
float64 area_min_x        # grid bounds (metres, world frame)
float64 area_min_y
float64 area_max_x
float64 area_max_y
float64 min_spacing       # centre-to-centre minimum; 0 => node default
int32   seed              # RNG seed; negative => nondeterministic
string  model_name        # spawnable model; "" => node default
---
# Response
bool         success
string       message
int32        requested
int32        placed
string[]     entity_names
```

- [ ] **Step 2: Create `package.xml`**

```xml
<?xml version="1.0"?>
<!--
Copyright 2026 amr-sim-lab contributors
Licensed under the Apache License, Version 2.0. See LICENSE.
-->
<package format="3">
  <name>scene_spawner_interfaces</name>
  <version>0.1.0</version>
  <description>Service definitions for scene_spawner.</description>
  <maintainer email="you@example.com">amr-sim-lab contributors</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <exec_depend>rosidl_default_runtime</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 3: Create `CMakeLists.txt`**

```cmake
# Copyright 2026 amr-sim-lab contributors
# Licensed under the Apache License, Version 2.0. See LICENSE.
cmake_minimum_required(VERSION 3.8)
project(scene_spawner_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SpawnLayout.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

- [ ] **Step 4: Local static check**

Run: `python3 -c "import xml.dom.minidom; xml.dom.minidom.parse('/workspace/simulation/src/scene_spawner_interfaces/package.xml'); print('ok')"`
Expected: `ok`
Run: `grep -c '^' /workspace/simulation/src/scene_spawner_interfaces/srv/SpawnLayout.srv`
Expected: a line count > 10, and manual eyeball that the `---` separator is present exactly once.

- [ ] **Step 5: Machine check (ROS box)**

Run: `cd /workspace/simulation && colcon build --packages-select scene_spawner_interfaces && source install/setup.bash && ros2 interface show scene_spawner_interfaces/srv/SpawnLayout`
Expected: build succeeds; the interface prints with all request and response fields.

- [ ] **Step 6: Commit**

```bash
cd /workspace/simulation
git add src/scene_spawner_interfaces/
git commit -m "feat: add SpawnLayout service interface

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Hmj5W4Ci2Nd8JvT6M9VMcG"
```

---

## Task 5: `spawner_node` — the ROS 2 spawner node

**Files:**
- Create: `src/scene_spawner/scene_spawner/spawner_node.py`
- Modify: `src/scene_spawner/package.xml` (deps already added in Task 1 — confirm)

**Interfaces:**
- Consumes:
  - `scene_spawner.layout_generator.generate_layout`, `LayoutParams`, `Placement`, `InvalidArea` (Task 2).
  - `scene_spawner.sdf_templates.include_spawn_sdf` (Task 3).
  - `scene_spawner_interfaces/srv/SpawnLayout` (Task 4).
  - `ros_gz_interfaces/srv/SpawnEntity`, `ros_gz_interfaces/srv/DeleteEntity`, `ros_gz_interfaces/msg/Entity` — **confirm exact names/types on the ROS box in Step 3** (`ros2 interface list | grep -i ros_gz`). If they differ, the node reads service names from parameters, so only the `srv` import and the request-field assignments change.
  - `std_srvs/srv/Trigger`, `geometry_msgs/msg/Pose`.
- Produces: an executable `spawner_node` (console script `scene_spawner spawner_node`). ROS services `~/spawn_layout` (`SpawnLayout`) and `~/clear_layout` (`Trigger`). Node name `scene_spawner`. Parameters: `world_name` (str `warehouse`), `default_count` (int `8`), `default_area` (double[4] `[-4,-4,4,4]`), `default_min_spacing` (double `1.0`), `default_model` (str `cart`), `cart_footprint_radius` (double `0.5`), `spawn_on_startup` (bool `true`), `startup_seed` (int `0`), `create_timeout_sec` (double `10.0`).

- [ ] **Step 1: Write `spawner_node.py`**

```python
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

    def _spawn_one(self, name: str, x: float, y: float, yaw: float, uri: str) -> bool:
        req = SpawnEntity.Request()
        req.entity_factory.name = name
        req.entity_factory.sdf = include_spawn_sdf(name, x, y, yaw, model_uri=uri)
        req.entity_factory.allow_renaming = False
        future = self._create_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        return bool(result and result.success)

    def _remove_one(self, name: str) -> bool:
        req = DeleteEntity.Request()
        req.entity.name = name
        req.entity.type = Entity.MODEL
        future = self._remove_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
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
        try:
            params = self._params_from_request(req)
        except InvalidArea as exc:
            resp.success = False
            resp.message = f"invalid area: {exc}"
            return resp

        if not self._wait_for_create():
            resp.success = False
            resp.message = f"Gazebo create service {self._create_name} unavailable"
            return resp

        with self._lock:
            self._clear_locked()
            placements = generate_layout(params, name_prefix=model)
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
```

- [ ] **Step 2: Local static check**

Run: `cd /workspace/simulation && python3 -m py_compile src/scene_spawner/scene_spawner/spawner_node.py && echo "compiles"`
Expected: `compiles`
Run: `cd /workspace/simulation/src/scene_spawner && python3 -m pytest test/test_no_ros_imports.py -v`
Expected: PASS (spawner_node.py is not in the pure list; this confirms the pure modules are still clean).

- [ ] **Step 3: Machine check (ROS box) — confirm the ros_gz interface surface**

Run: `source /opt/ros/humble/setup.bash && ros2 interface list | grep -i ros_gz`
Expected: `ros_gz_interfaces/srv/SpawnEntity` and `ros_gz_interfaces/srv/DeleteEntity` present.
Run: `ros2 interface show ros_gz_interfaces/srv/SpawnEntity` and `... DeleteEntity`.
Confirm: `SpawnEntity.Request` has `entity_factory` of type `ros_gz_interfaces/EntityFactory` with fields `name`, `sdf`, `allow_renaming`; `DeleteEntity.Request` has `entity` of type `ros_gz_interfaces/Entity` with `name` and `type`, and `Entity` has a `MODEL` constant.
If any differ, adjust the request-field assignments in `_spawn_one` / `_remove_one` accordingly and note the change in the commit message.

- [ ] **Step 4: Machine check (ROS box) — build**

Run: `cd /workspace/simulation && colcon build --packages-select scene_spawner_interfaces scene_spawner && source install/setup.bash && ros2 run scene_spawner spawner_node --ros-args -p spawn_on_startup:=false`
Expected: node starts, logs no errors, `ros2 service list` shows `/scene_spawner/spawn_layout` and `/scene_spawner/clear_layout`. Ctrl-C to stop. (Full spawn behavior is exercised in Task 14.)

- [ ] **Step 5: Commit**

```bash
cd /workspace/simulation
git add src/scene_spawner/scene_spawner/spawner_node.py src/scene_spawner/package.xml
git commit -m "feat: add scene_spawner ROS 2 node

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Hmj5W4Ci2Nd8JvT6M9VMcG"
```

---

## Task 6: `safety_stop` — pure gate + ROS node

**Files:**
- Create: `src/safety_stop/safety_stop/gate.py`
- Create: `src/safety_stop/safety_stop/safety_stop_node.py`
- Test: `src/safety_stop/test/test_gate.py`
- Test: `src/safety_stop/test/test_no_ros_imports.py`

**Interfaces:**
- Consumes: `sensor_msgs/msg/LaserScan`, `geometry_msgs/msg/Twist` (node only).
- Produces:
  - `gate.obstacle_in_front(ranges: Sequence[float], angle_min: float, angle_increment: float, range_min: float, range_max: float, stop_distance: float, front_arc_rad: float) -> bool`
  - `gate.apply_gate(linear_x: float, angular_z: float, blocked: bool) -> tuple[float, float]`
  - executable `safety_stop_node` (console script `safety_stop safety_stop_node`). Subscribes `/cmd_vel_raw` (`Twist`), `/scan` (`LaserScan`). Publishes `/cmd_vel` (`Twist`). Node name `safety_stop`. Parameters: `stop_distance` (double `0.6`), `front_arc_deg` (double `90.0`), `require_scan` (bool `true`), `scan_timeout_sec` (double `1.0`).

- [ ] **Step 1: Write the failing tests**

`src/safety_stop/test/test_gate.py`:

```python
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

import math

from safety_stop.gate import apply_gate, obstacle_in_front

# 180-degree scan, 1-degree resolution, index 90 == straight ahead (angle 0).
ANGLE_MIN = -math.pi / 2
ANGLE_INC = math.radians(1.0)
RANGE_MIN = 0.05
RANGE_MAX = 12.0
STOP = 0.6
ARC = math.radians(90.0)


def _clear_scan():
    return [10.0] * 181


def test_clear_path_not_blocked():
    assert obstacle_in_front(_clear_scan(), ANGLE_MIN, ANGLE_INC,
                             RANGE_MIN, RANGE_MAX, STOP, ARC) is False


def test_obstacle_dead_ahead_blocks():
    scan = _clear_scan()
    scan[90] = 0.3
    assert obstacle_in_front(scan, ANGLE_MIN, ANGLE_INC,
                             RANGE_MIN, RANGE_MAX, STOP, ARC) is True


def test_obstacle_behind_does_not_block():
    # A 180-degree forward scan cannot see behind; simulate a rear return
    # by widening: place a close return at the extreme edge, outside the arc.
    scan = _clear_scan()
    scan[0] = 0.2      # angle -90 deg, outside +/-45 deg arc
    scan[180] = 0.2    # angle +90 deg, outside arc
    assert obstacle_in_front(scan, ANGLE_MIN, ANGLE_INC,
                             RANGE_MIN, RANGE_MAX, STOP, ARC) is False


def test_obstacle_just_outside_arc_ignored():
    scan = _clear_scan()
    scan[44] = 0.3     # angle -46 deg, just outside +/-45 deg
    assert obstacle_in_front(scan, ANGLE_MIN, ANGLE_INC,
                             RANGE_MIN, RANGE_MAX, STOP, ARC) is False


def test_invalid_returns_ignored():
    scan = [math.nan, math.inf, -1.0, 999.0] + [10.0] * 177
    assert obstacle_in_front(scan, ANGLE_MIN, ANGLE_INC,
                             RANGE_MIN, RANGE_MAX, STOP, ARC) is False


def test_obstacle_beyond_stop_distance_ignored():
    scan = _clear_scan()
    scan[90] = 0.9     # inside arc, but past 0.6 m stop distance
    assert obstacle_in_front(scan, ANGLE_MIN, ANGLE_INC,
                             RANGE_MIN, RANGE_MAX, STOP, ARC) is False


def test_apply_gate_blocks_forward_only():
    assert apply_gate(0.5, 0.3, blocked=True) == (0.0, 0.3)


def test_apply_gate_allows_reverse_when_blocked():
    assert apply_gate(-0.4, 0.0, blocked=True) == (-0.4, 0.0)


def test_apply_gate_passes_through_when_clear():
    assert apply_gate(0.5, -0.2, blocked=False) == (0.5, -0.2)
```

`src/safety_stop/test/test_no_ros_imports.py`: identical to the Task 2 version but with `PURE_MODULES = ["gate.py"]` and the package path pointing at `safety_stop/`.

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd /workspace/simulation/src/safety_stop && python3 -m pytest test/ -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'safety_stop.gate'`

- [ ] **Step 3: Implement `gate.py`**

```python
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

"""Pure obstacle-gating logic for the safety-stop node. No ROS imports."""

from __future__ import annotations

import math
from collections.abc import Sequence


def obstacle_in_front(ranges: Sequence[float], angle_min: float,
                      angle_increment: float, range_min: float, range_max: float,
                      stop_distance: float, front_arc_rad: float) -> bool:
    """True if a valid return within +/- ``front_arc_rad`` / 2 of straight
    ahead (bearing 0) is closer than ``stop_distance``."""
    half_arc = front_arc_rad / 2.0
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r < range_min or r > range_max:
            continue
        bearing = angle_min + i * angle_increment
        bearing = math.atan2(math.sin(bearing), math.cos(bearing))
        if abs(bearing) <= half_arc and r < stop_distance:
            return True
    return False


def apply_gate(linear_x: float, angular_z: float, blocked: bool) -> tuple[float, float]:
    """Clamp forward motion when ``blocked``. Rotation and reverse pass through."""
    if blocked and linear_x > 0.0:
        return 0.0, angular_z
    return linear_x, angular_z
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd /workspace/simulation/src/safety_stop && python3 -m pytest test/ -v`
Expected: PASS — `test_gate.py` and `test_no_ros_imports.py`.

- [ ] **Step 5: Implement `safety_stop_node.py`**

```python
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

"""ROS 2 node: gate /cmd_vel_raw -> /cmd_vel using a forward lidar arc."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from safety_stop.gate import apply_gate, obstacle_in_front


class SafetyStopNode(Node):

    def __init__(self) -> None:
        super().__init__("safety_stop")
        self.declare_parameter("stop_distance", 0.6)
        self.declare_parameter("front_arc_deg", 90.0)
        self.declare_parameter("require_scan", True)
        self.declare_parameter("scan_timeout_sec", 1.0)

        self._last_scan: LaserScan | None = None
        self._last_scan_time = self.get_clock().now()

        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.create_subscription(Twist, "/cmd_vel_raw", self._on_cmd, 10)
        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def _on_scan(self, msg: LaserScan) -> None:
        self._last_scan = msg
        self._last_scan_time = self.get_clock().now()

    def _scan_is_fresh(self) -> bool:
        if self._last_scan is None:
            return False
        age = (self.get_clock().now() - self._last_scan_time).nanoseconds / 1e9
        return age <= float(self.get_parameter("scan_timeout_sec").value)

    def _on_cmd(self, msg: Twist) -> None:
        out = Twist()
        out.angular.z = msg.angular.z
        out.linear.x = msg.linear.x

        if not self._scan_is_fresh():
            if bool(self.get_parameter("require_scan").value):
                self._pub.publish(Twist())  # fail-safe: full stop
                self.get_logger().warn("no fresh /scan; holding robot",
                                       throttle_duration_sec=2.0)
                return
            self._pub.publish(out)
            return

        scan = self._last_scan
        blocked = obstacle_in_front(
            scan.ranges, scan.angle_min, scan.angle_increment,
            scan.range_min, scan.range_max,
            float(self.get_parameter("stop_distance").value),
            math.radians(float(self.get_parameter("front_arc_deg").value)),
        )
        lin, ang = apply_gate(msg.linear.x, msg.angular.z, blocked)
        out.linear.x, out.angular.z = lin, ang
        if blocked and msg.linear.x > 0.0:
            self.get_logger().warn("obstacle within stop distance; blocking forward",
                                   throttle_duration_sec=2.0)
        self._pub.publish(out)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SafetyStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 6: Local static check**

Run: `cd /workspace/simulation && python3 -m py_compile src/safety_stop/safety_stop/safety_stop_node.py && echo compiles`
Expected: `compiles`

- [ ] **Step 7: Machine check (ROS box)**

Run: `cd /workspace/simulation && colcon build --packages-select safety_stop && source install/setup.bash && ros2 run safety_stop safety_stop_node`
In another terminal: `ros2 topic pub -1 /cmd_vel_raw geometry_msgs/msg/Twist "{linear: {x: 0.5}}"` — with no `/scan` and `require_scan:=true`, `ros2 topic echo /cmd_vel` shows a zero Twist.
Then: `ros2 topic pub -r 10 /scan sensor_msgs/msg/LaserScan "{angle_min: -1.57, angle_max: 1.57, angle_increment: 0.0174, range_min: 0.05, range_max: 12.0, ranges: [10.0, 10.0, ...]}"` and re-publish the cmd — `/cmd_vel` now passes `x: 0.5` through.

- [ ] **Step 8: Commit**

```bash
cd /workspace/simulation
git add src/safety_stop/
git commit -m "feat: add safety_stop gate and node

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Hmj5W4Ci2Nd8JvT6M9VMcG"
```

---

## Task 7: `sim_bringup` config — bridge and layout

**Files:**
- Create: `src/sim_bringup/config/bridge.yaml`
- Create: `src/sim_bringup/config/layout.yaml`
- Create: `config/layout.yaml` (symlink → `../src/sim_bringup/config/layout.yaml`)

**Interfaces:**
- Consumes: parameter names from Task 5 (`scene_spawner` node).
- Produces: `bridge.yaml` (a `ros_gz_bridge` `config_file`), `layout.yaml` (a ROS 2 params file loaded by the spawner node), and a repo-root `config/` pointer for Phase 2 tooling.

- [ ] **Step 1: Create `src/sim_bringup/config/layout.yaml`**

```yaml
# Copyright 2026 amr-sim-lab contributors
# Licensed under the Apache License, Version 2.0. See LICENSE.
#
# Canonical scene definition. Phase 2 (Isaac Sim) tooling reads this file too.
/scene_spawner:
  ros__parameters:
    world_name: warehouse
    default_count: 8
    default_area: [-4.0, -4.0, 4.0, 4.0]
    default_min_spacing: 1.0
    default_model: cart
    cart_footprint_radius: 0.5
    spawn_on_startup: true
    startup_seed: 0
    create_timeout_sec: 10.0
```

- [ ] **Step 2: Create `src/sim_bringup/config/bridge.yaml`**

`ros_gz_bridge` parameter-bridge config. **Confirm topic names on the ROS box in Step 5** — the robot's lidar and odom topics depend on the Dolly SDF (Task 11). Placeholders marked below.

```yaml
# Copyright 2026 amr-sim-lab contributors
# Licensed under the Apache License, Version 2.0. See LICENSE.
- ros_topic_name: "/clock"
  gz_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS

- ros_topic_name: "/cmd_vel"
  gz_topic_name: "/model/dolly/cmd_vel"          # confirm against Dolly SDF
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ

- ros_topic_name: "/odom"
  gz_topic_name: "/model/dolly/odometry"         # confirm against Dolly SDF
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS

- ros_topic_name: "/scan"
  gz_topic_name: "/lidar"                         # confirm against Dolly SDF
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS

- ros_topic_name: "/tf"
  gz_topic_name: "/model/dolly/pose"             # confirm against Dolly SDF
  ros_type_name: "tf2_msgs/msg/TFMessage"
  gz_type_name: "gz.msgs.Pose_V"
  direction: GZ_TO_ROS
```

- [ ] **Step 3: Create the repo-root symlink**

```bash
cd /workspace/simulation
mkdir -p config
ln -sf ../src/sim_bringup/config/layout.yaml config/layout.yaml
```

- [ ] **Step 4: Local static check**

Run:
```bash
python3 -c "import yaml; yaml.safe_load(open('/workspace/simulation/src/sim_bringup/config/layout.yaml')); yaml.safe_load(open('/workspace/simulation/src/sim_bringup/config/bridge.yaml')); print('yaml ok')"
```
Expected: `yaml ok`
Run: `readlink /workspace/simulation/config/layout.yaml`
Expected: `../src/sim_bringup/config/layout.yaml`

- [ ] **Step 5: Machine check (ROS box) — deferred to Task 8**

Bridge correctness is verified when the full launch runs (Task 8, Step 4). Note here any topic-name corrections discovered then and fix `bridge.yaml`.

- [ ] **Step 6: Commit**

```bash
cd /workspace/simulation
git add src/sim_bringup/config/ config/layout.yaml
git commit -m "feat: add sim_bringup bridge and layout config

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Hmj5W4Ci2Nd8JvT6M9VMcG"
```

---

## Task 8: `sim_bringup` launch files

**Files:**
- Create: `src/sim_bringup/launch/spawner_only.launch.py`
- Create: `src/sim_bringup/launch/sim.launch.py`

**Interfaces:**
- Consumes: `scene_spawner` executable (Task 5), `safety_stop` executable (Task 6), `bridge.yaml` + `layout.yaml` (Task 7), `worlds/warehouse.sdf` (Task 9), `models/` (Tasks 10–11).
- Produces: two launchable entry points. `spawner_only.launch.py` args: `world` (default `warehouse`), `headless` (default `true`). `sim.launch.py` args: `world` (default `warehouse`), `gui` (default `true`), `teleop` (default `false`), `robot` (default `dolly`).

- [ ] **Step 1: Write `spawner_only.launch.py`**

```python
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

"""Headless launch: Gazebo server + spawner node only (for tests/iteration)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("sim_bringup")
    repo_root = os.path.abspath(os.path.join(pkg, "..", "..", "..", ".."))
    models_path = os.path.join(repo_root, "models")
    worlds_path = os.path.join(repo_root, "worlds")

    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")

    layout_yaml = PathJoinSubstitution([FindPackageShare("sim_bringup"),
                                        "config", "layout.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="warehouse"),
        DeclareLaunchArgument("headless", default_value="true"),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH",
                               f"{models_path}:{worlds_path}"),

        Node(
            package="ros_gz_sim", executable="gz_sim",
            output="screen",
            arguments=[[worlds_path, "/", world, ".sdf"],
                       "-r", "-s", "--headless-rendering"],
            condition=None,
        ),
        Node(
            package="scene_spawner", executable="spawner_node",
            name="scene_spawner", output="screen",
            parameters=[layout_yaml, {"world_name": world}],
        ),
    ])
```

> Note: `ros_gz_sim`'s launch-friendly entry is normally the included `gz_sim.launch.py`. If `executable="gz_sim"` is not resolvable on the ROS box, replace the Gazebo `Node` with an `IncludeLaunchDescription` of `os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")` passing `gz_args`. Confirm in Step 3 and adjust.

- [ ] **Step 2: Write `sim.launch.py`**

```python
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

"""Full demo launch: Gazebo + bridge + robot + spawner + safety_stop."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("sim_bringup")
    repo_root = os.path.abspath(os.path.join(pkg, "..", "..", "..", ".."))
    models_path = os.path.join(repo_root, "models")
    worlds_path = os.path.join(repo_root, "worlds")

    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    teleop = LaunchConfiguration("teleop")
    robot = LaunchConfiguration("robot")

    layout_yaml = PathJoinSubstitution([FindPackageShare("sim_bringup"),
                                        "config", "layout.yaml"])
    bridge_yaml = PathJoinSubstitution([FindPackageShare("sim_bringup"),
                                        "config", "bridge.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="warehouse"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("teleop", default_value="false"),
        DeclareLaunchArgument("robot", default_value="dolly"),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH",
                               f"{models_path}:{worlds_path}"),

        Node(
            package="ros_gz_sim", executable="gz_sim", output="screen",
            arguments=[[worlds_path, "/", world, ".sdf"], "-r"],
        ),
        Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            output="screen",
            parameters=[{"config_file": bridge_yaml}],
        ),
        Node(
            package="ros_gz_sim", executable="create", output="screen",
            arguments=["-world", world, "-name", robot,
                       "-file", [models_path, "/", robot, "/model.sdf"],
                       "-x", "0", "-y", "0", "-z", "0.1"],
        ),
        Node(
            package="scene_spawner", executable="spawner_node",
            name="scene_spawner", output="screen",
            parameters=[layout_yaml, {"world_name": world}],
        ),
        Node(
            package="safety_stop", executable="safety_stop_node",
            name="safety_stop", output="screen",
        ),
        Node(
            package="teleop_twist_keyboard", executable="teleop_twist_keyboard",
            name="teleop", output="screen", prefix="xterm -e",
            remappings=[("/cmd_vel", "/cmd_vel_raw")],
            condition=IfCondition(teleop),
        ),
    ])
```

- [ ] **Step 3: Local static check**

Run: `cd /workspace/simulation && python3 -m py_compile src/sim_bringup/launch/spawner_only.launch.py src/sim_bringup/launch/sim.launch.py && echo compiles`
Expected: `compiles`

- [ ] **Step 4: Machine check (ROS box)**

Run: `cd /workspace/simulation && colcon build && source install/setup.bash && ros2 launch sim_bringup sim.launch.py`
Expected: Gazebo opens with the warehouse world, the robot appears, ~8 carts appear around it, `ros2 topic list` shows `/scan`, `/cmd_vel`, `/odom`, `/clock`. Fix any `bridge.yaml` topic names or the `gz_sim` executable name per the notes in Tasks 7–8; re-run until clean; amend the relevant commits or add a `fix:` commit.

- [ ] **Step 5: Commit**

```bash
cd /workspace/simulation
git add src/sim_bringup/launch/
git commit -m "feat: add spawner_only and full sim launch files

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Hmj5W4Ci2Nd8JvT6M9VMcG"
```

---

## Task 9: `worlds/warehouse.sdf`

**Files:**
- Create: `worlds/warehouse.sdf`

**Interfaces:**
- Consumes: nothing.
- Produces: a Gazebo Harmonic world named `warehouse` (the `<world name="...">` must match the `world_name` param default). Contains ground, a ~8 m × 8 m low-wall perimeter, one directional + ambient light, and the systems `Physics`, `UserCommands`, `SceneBroadcaster`, `Sensors`, `Contact`.

- [ ] **Step 1: Write `worlds/warehouse.sdf`**

```xml
<?xml version="1.0" ?>
<!--
Copyright 2026 amr-sim-lab contributors
Licensed under the Apache License, Version 2.0. See LICENSE.
-->
<sdf version="1.10">
  <world name="warehouse">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-contact-system"
            name="gz::sim::systems::Contact"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
    </light>
    <scene>
      <ambient>0.5 0.5 0.5 1</ambient>
      <background>0.75 0.78 0.82 1</background>
    </scene>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="perimeter">
      <static>true</static>
      <link name="link">
        <!-- four 8.4 m x 0.2 m x 0.5 m walls around a 8x8 interior -->
        <collision name="n"><pose>0 4.1 0.25 0 0 0</pose>
          <geometry><box><size>8.4 0.2 0.5</size></box></geometry></collision>
        <visual name="nv"><pose>0 4.1 0.25 0 0 0</pose>
          <geometry><box><size>8.4 0.2 0.5</size></box></geometry></visual>
        <collision name="s"><pose>0 -4.1 0.25 0 0 0</pose>
          <geometry><box><size>8.4 0.2 0.5</size></box></geometry></collision>
        <visual name="sv"><pose>0 -4.1 0.25 0 0 0</pose>
          <geometry><box><size>8.4 0.2 0.5</size></box></geometry></visual>
        <collision name="e"><pose>4.1 0 0.25 0 0 0</pose>
          <geometry><box><size>0.2 8.4 0.5</size></box></geometry></collision>
        <visual name="ev"><pose>4.1 0 0.25 0 0 0</pose>
          <geometry><box><size>0.2 8.4 0.5</size></box></geometry></visual>
        <collision name="w"><pose>-4.1 0 0.25 0 0 0</pose>
          <geometry><box><size>0.2 8.4 0.5</size></box></geometry></collision>
        <visual name="wv"><pose>-4.1 0 0.25 0 0 0</pose>
          <geometry><box><size>0.2 8.4 0.5</size></box></geometry></visual>
      </link>
    </model>
  </world>
</sdf>
```

- [ ] **Step 2: Local static check**

Run: `python3 -c "import xml.etree.ElementTree as ET; r=ET.parse('/workspace/simulation/worlds/warehouse.sdf').getroot(); assert r.find('world').attrib['name']=='warehouse'; print('world ok')"`
Expected: `world ok`

- [ ] **Step 3: Machine check (ROS box)**

Run: `gz sim -s -r --iterations 200 /workspace/simulation/worlds/warehouse.sdf`
Expected: runs 200 steps and exits with no plugin-load or parse errors. Then `gz sim /workspace/simulation/worlds/warehouse.sdf` (GUI) shows the floor and four walls.

- [ ] **Step 4: Commit**

```bash
cd /workspace/simulation
git add worlds/warehouse.sdf
git commit -m "feat: add warehouse world

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Hmj5W4Ci2Nd8JvT6M9VMcG"
```

---

## Task 10: `models/cart/` — the spawnable cart

**Files:**
- Create: `models/cart/model.config`
- Create: `models/cart/model.sdf`

**Interfaces:**
- Consumes: nothing.
- Produces: a Gazebo model named `cart`, resolvable as `model://cart` when `models/` is on `GZ_SIM_RESOURCE_PATH`. Dynamic rigid body, primitive geometry, ~20 kg. Footprint ≈ 0.8 m × 0.5 m (fits the `cart_footprint_radius: 0.5` default).

> **Deviation from spec §4.2:** the spec calls for a glTF (`.glb`) visual mesh. This task uses primitive box/cylinder geometry instead — no art asset to source or author, and USD conversion in Phase 2 is still clean (USD physics has native box/cylinder prims). Sourcing a `.glb` is tracked as Phase 2 polish.

- [ ] **Step 1: Write `models/cart/model.config`**

```xml
<?xml version="1.0"?>
<!-- Copyright 2026 amr-sim-lab contributors. Apache-2.0. See LICENSE. -->
<model>
  <name>cart</name>
  <version>1.0</version>
  <sdf version="1.10">model.sdf</sdf>
  <description>Primitive warehouse cart prop (dynamic, ~20 kg).</description>
</model>
```

- [ ] **Step 2: Write `models/cart/model.sdf`**

```xml
<?xml version="1.0" ?>
<!-- Copyright 2026 amr-sim-lab contributors. Apache-2.0. See LICENSE. -->
<sdf version="1.10">
  <model name="cart">
    <link name="body">
      <inertial>
        <mass>20.0</mass>
        <inertia>
          <ixx>1.2</ixx><iyy>1.7</iyy><izz>2.4</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="deck">
        <pose>0 0 0.5 0 0 0</pose>
        <geometry><box><size>0.8 0.5 0.1</size></box></geometry>
      </collision>
      <visual name="deck">
        <pose>0 0 0.5 0 0 0</pose>
        <geometry><box><size>0.8 0.5 0.1</size></box></geometry>
        <material><ambient>0.2 0.3 0.6 1</ambient><diffuse>0.2 0.3 0.6 1</diffuse></material>
      </visual>
      <visual name="post_fl"><pose>0.35 0.2 0.25 0 0 0</pose>
        <geometry><cylinder><radius>0.02</radius><length>0.5</length></cylinder></geometry></visual>
      <visual name="post_fr"><pose>0.35 -0.2 0.25 0 0 0</pose>
        <geometry><cylinder><radius>0.02</radius><length>0.5</length></cylinder></geometry></visual>
      <visual name="post_bl"><pose>-0.35 0.2 0.25 0 0 0</pose>
        <geometry><cylinder><radius>0.02</radius><length>0.5</length></cylinder></geometry></visual>
      <visual name="post_br"><pose>-0.35 -0.2 0.25 0 0 0</pose>
        <geometry><cylinder><radius>0.02</radius><length>0.5</length></cylinder></geometry></visual>
      <collision name="legs">
        <pose>0 0 0.25 0 0 0</pose>
        <geometry><box><size>0.7 0.4 0.5</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>
```

- [ ] **Step 3: Local static check**

Run: `python3 -c "import xml.etree.ElementTree as ET; [ET.parse(p) for p in ['/workspace/simulation/models/cart/model.config','/workspace/simulation/models/cart/model.sdf']]; print('cart xml ok')"`
Expected: `cart xml ok`

- [ ] **Step 4: Machine check (ROS box)**

Run: `GZ_SIM_RESOURCE_PATH=/workspace/simulation/models gz sim -s -r --iterations 300 /workspace/simulation/worlds/warehouse.sdf` then in another shell:
`gz service -s /world/warehouse/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 2000 --req 'sdf: "<?xml version=\"1.0\"?><sdf version=\"1.10\"><include><name>cart_test</name><uri>model://cart</uri><pose>1 1 0 0 0 0</pose></include></sdf>"'`
Expected: `data: true`; the cart appears (verify with `gz model --list`).

- [ ] **Step 5: Commit**

```bash
cd /workspace/simulation
git add models/cart/
git commit -m "feat: add primitive cart model

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Hmj5W4Ci2Nd8JvT6M9VMcG"
```

---

## Task 11: Robot model — vendor Dolly

**Files:**
- Create: `.gitmodules` (via `git submodule add`)
- Create: `models/dolly/` (submodule checkout)
- Modify: `NOTICE` (already lists Dolly — confirm)
- Possibly modify: `src/sim_bringup/config/bridge.yaml`, `src/sim_bringup/launch/sim.launch.py` (topic / path corrections)

**Interfaces:**
- Consumes: `GZ_SIM_RESOURCE_PATH` set by the launch files (Task 8).
- Produces: a spawnable robot at `models/dolly/` exposing, in Gazebo, a differential-drive `cmd_vel` input, an odometry output, and a 2D lidar. The exact gz topic names are recorded into `bridge.yaml` in Step 3.

- [ ] **Step 1: Add the submodule**

```bash
cd /workspace/simulation
git submodule add https://github.com/chapulina/dolly models/dolly
git submodule update --init --recursive
```

Locate the model directory that contains a `model.config` (likely `models/dolly/models/dolly` or `models/dolly/dolly_gazebo/models/dolly`). If it is nested, add that parent to `GZ_SIM_RESOURCE_PATH` in both launch files instead of `models/` — or create `models/dolly_model` as a path pointer. Record the resolved model path in a comment at the top of `sim.launch.py`.

- [ ] **Step 2: Local static check**

Run: `test -f /workspace/simulation/models/dolly/.git && find /workspace/simulation/models/dolly -name model.config`
Expected: at least one `model.config` path printed.
Run: `python3 -c "import configparser" ` (noop) then eyeball the found `model.sdf` for `<sensor` `type="gpu_lidar"` (or `ray`) and a `DiffDrive` plugin.

- [ ] **Step 3: Machine check (ROS box) — resolve topics**

Run: `GZ_SIM_RESOURCE_PATH=<resolved dolly parent>:/workspace/simulation/models gz sim -r <dolly world or warehouse + create>` and inspect:
`gz topic -l | grep -Ei 'dolly|lidar|scan|odom|cmd_vel'`
Record the actual topic names and update `bridge.yaml` (`gz_topic_name` fields) and the `create -file` path in `sim.launch.py`. If Dolly does not load on Harmonic at all, use **Appendix A** (minimal `diffbot` SDF) instead and set `robot` default to `diffbot`.

- [ ] **Step 4: Machine check (ROS box) — drive it**

Run: `ros2 launch sim_bringup sim.launch.py teleop:=true`
Expected: drive the robot with the keyboard; `/scan` populates in `ros2 topic echo /scan --once`; driving straight at a cart, the robot stops ~0.6 m short (safety_stop working end to end).

- [ ] **Step 5: Commit**

```bash
cd /workspace/simulation
git add .gitmodules models/dolly src/sim_bringup/
git commit -m "feat: vendor Dolly robot and wire bridge topics

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Hmj5W4Ci2Nd8JvT6M9VMcG"
```

---

## Task 12: Docker + devcontainer

**Files:**
- Create: `docker/Dockerfile`
- Create: `docker/entrypoint.sh`
- Create: `.devcontainer/devcontainer.json`

**Interfaces:**
- Consumes: the whole workspace.
- Produces: an image with ROS 2 Humble + Gazebo Harmonic + `ros-humble-ros-gzharmonic` + build tools, and a devcontainer that opens the repo in it.

- [ ] **Step 1: Write `docker/Dockerfile`**

```dockerfile
# Copyright 2026 amr-sim-lab contributors
# Licensed under the Apache License, Version 2.0. See LICENSE.
FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-c"]

# Gazebo Harmonic via the OSRF apt repo + the ros_gz build against it.
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl gnupg lsb-release \
    && curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        gz-harmonic \
        ros-humble-ros-gzharmonic \
        ros-humble-teleop-twist-keyboard \
        ros-humble-rmw-cyclonedds-cpp \
        python3-colcon-common-extensions \
        python3-pytest \
        build-essential \
        xterm \
    && rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
WORKDIR /ws
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
```

- [ ] **Step 2: Write `docker/entrypoint.sh`**

```bash
#!/usr/bin/env bash
# Copyright 2026 amr-sim-lab contributors. Apache-2.0. See LICENSE.
set -e
source /opt/ros/humble/setup.bash
if [ -f /ws/install/setup.bash ]; then
  source /ws/install/setup.bash
fi
exec "$@"
```

- [ ] **Step 3: Write `.devcontainer/devcontainer.json`**

```json
{
  "name": "amr-sim-lab",
  "build": { "dockerfile": "../docker/Dockerfile", "context": ".." },
  "runArgs": ["--net=host", "--gpus", "all", "-e", "DISPLAY"],
  "mounts": [
    "source=/tmp/.X11-unix,target=/tmp/.X11-unix,type=bind"
  ],
  "workspaceFolder": "/ws",
  "workspaceMount": "source=${localWorkspaceFolder},target=/ws,type=bind",
  "customizations": {
    "vscode": {
      "extensions": ["ms-python.python", "ms-iot.vscode-ros"]
    }
  }
}
```

- [ ] **Step 4: Local static check**

Run: `python3 -c "import json; json.load(open('/workspace/simulation/.devcontainer/devcontainer.json')); print('json ok')"`
Expected: `json ok`
Run: `bash -n /workspace/simulation/docker/entrypoint.sh && echo "sh ok"`
Expected: `sh ok`

- [ ] **Step 5: Machine check (user machine with Docker)**

Run: `cd /workspace/simulation && docker build -f docker/Dockerfile -t amr-sim-lab .`
Expected: image builds. Then `docker run --rm -it --net=host amr-sim-lab bash -lc "colcon build && colcon test && colcon test-result --verbose"` — build + unit tests pass inside the container.

- [ ] **Step 6: Commit**

```bash
cd /workspace/simulation
git add docker/ .devcontainer/
git commit -m "feat: add Docker image and devcontainer

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Hmj5W4Ci2Nd8JvT6M9VMcG"
```

---

## Task 13: Makefile, demo script, README, docs

**Files:**
- Create: `Makefile`, `scripts/demo.sh`
- Create: `README.md`, `docs/gazebo-vs-isaac.md`, `docs/smoke-test.md`

**Interfaces:**
- Consumes: all prior tasks.
- Produces: `make build|test|test-integration|run|clean`; a one-command demo script; user-facing docs.

- [ ] **Step 1: Write `Makefile`**

```makefile
# Copyright 2026 amr-sim-lab contributors. Apache-2.0. See LICENSE.
.PHONY: build test test-integration run clean

build:
	colcon build --symlink-install

test:
	cd src/scene_spawner && python3 -m pytest test/ -v
	cd src/safety_stop && python3 -m pytest test/ -v

test-integration:
	colcon build --symlink-install
	. install/setup.bash && python3 -m pytest src/sim_bringup/test/ -v

run:
	. install/setup.bash && ros2 launch sim_bringup sim.launch.py teleop:=true

clean:
	rm -rf build install log
```

- [ ] **Step 2: Write `scripts/demo.sh`**

```bash
#!/usr/bin/env bash
# Copyright 2026 amr-sim-lab contributors. Apache-2.0. See LICENSE.
set -euo pipefail
cd "$(dirname "$0")/.."
colcon build --symlink-install
# shellcheck disable=SC1091
source install/setup.bash
ros2 launch sim_bringup sim.launch.py &
LAUNCH_PID=$!
sleep 15
ros2 service call /scene_spawner/spawn_layout \
  scene_spawner_interfaces/srv/SpawnLayout "{count: 12, seed: 7}"
wait $LAUNCH_PID
```

- [ ] **Step 3: Write `README.md`**

Sections, in order:
1. **Title + one-paragraph description** — what it is (procedural randomized cart spawner + drivable lidar AMR in ROS 2 Humble / Gazebo Harmonic), no employer/job references.
2. **Demo GIF** placeholder: `![demo](docs/images/demo.gif)` (GIF captured later, non-blocking).
3. **Architecture** — the 4 packages, one line each; link `docs/superpowers/specs/2026-09-08-amr-sim-lab-design.md`.
4. **Requirements** — Ubuntu 22.04, ROS 2 Humble, Gazebo Harmonic (`ros-humble-ros-gzharmonic`), or just Docker. Explicit note: "Humble ships with Fortress by default; this project uses Harmonic, installed via `ros-humble-ros-gzharmonic`."
5. **Quickstart (Docker)** — `docker build -f docker/Dockerfile -t amr-sim-lab .` then the `docker run` line, then `make run`.
6. **Quickstart (native)** — `make build && make run`.
7. **Usage** — the `/scene_spawner/spawn_layout` and `/clear_layout` service calls with example payloads; teleop keys; `robot:=` / `teleop:=` / `gui:=` launch args.
8. **Testing** — `make test` (unit, no ROS graph needed beyond the Python pkgs), `make test-integration` (needs the built workspace).
9. **Phase 2 roadmap** — port to NVIDIA Isaac Sim + OpenUSD + Replicator: reuse `layout_generator` + `config/layout.yaml`, convert the cart to USD, drive the robot over the Isaac Sim ROS 2 bridge, add one Isaac Lab RL task. Link `docs/gazebo-vs-isaac.md`.
10. **License** — Apache-2.0; Dolly attribution.

- [ ] **Step 4: Write `docs/gazebo-vs-isaac.md`**

A living comparison doc with these headed sections (fill each with 2–4 bullets of current understanding):
- Scene description: SDF `<world>`/`<model>`/`<include>` vs USD stage / prims / references / composition arcs
- Physics: Gazebo DART (CPU) vs Isaac PhysX (GPU), fixed vs GPU-batched stepping
- Extensibility: gz-sim systems (plugins per world/model) vs Omniverse Kit extensions
- Sensors & synthetic data: gz sensors + bridge vs Isaac Replicator annotators/writers/randomizers
- ROS bridge: `ros_gz_bridge` parameter_bridge vs Isaac Sim ROS 2 bridge (OmniGraph action graph)
- Scale: one world / one real-time factor vs Isaac Lab thousands of vectorized envs on one GPU
- What ports directly from this repo: `layout_generator.py`, `config/layout.yaml`, cart dimensions

- [ ] **Step 5: Write `docs/smoke-test.md`**

A manual checklist: build; `ros2 launch sim_bringup sim.launch.py`; confirm world + robot + ~8 carts; `ros2 service call .../spawn_layout "{count: 15, seed: 3}"` twice with the same seed → identical layout; `teleop:=true`, drive into a cart → robot halts; `/clear_layout` → carts gone.

- [ ] **Step 6: Local static check**

Run: `bash -n /workspace/simulation/scripts/demo.sh && make -n -C /workspace/simulation test && echo ok`
Expected: `ok` (the `make -n` dry-run prints the pytest commands).
Run: `cd /workspace/simulation && make test`
Expected: PASS — all pure unit tests across `scene_spawner` and `safety_stop`.

- [ ] **Step 7: Commit**

```bash
cd /workspace/simulation
git add Makefile scripts/ README.md docs/gazebo-vs-isaac.md docs/smoke-test.md
git commit -m "docs: add README, Makefile, demo script, and comparison notes

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Hmj5W4Ci2Nd8JvT6M9VMcG"
```

---

## Task 14: Integration test — spawn round-trip

**Files:**
- Create: `src/sim_bringup/test/test_spawn_integration.py`
- Modify: `src/sim_bringup/package.xml` (add `<test_depend>launch_testing_ament_cmake</test_depend>` — for ament_python use `<test_depend>python3-pytest</test_depend>` and `<test_depend>launch_testing</test_depend>`)

**Interfaces:**
- Consumes: `spawner_only.launch.py` (Task 8), `SpawnLayout` srv (Task 4), the `cart` model (Task 10), `warehouse.sdf` (Task 9).
- Produces: an automated test (local / CI-with-gz only) asserting the spawn/clear round-trip.

- [ ] **Step 1: Write `src/sim_bringup/test/test_spawn_integration.py`**

```python
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
```

> **launch_testing variant (preferred if `ros_gz_sim` launch integration is confirmed in Task 8):** wrap the above with `launch_testing.main`, having `generate_test_description()` `IncludeLaunchDescription(spawner_only.launch.py)` + `ReadyToTest()`, so the test owns Gazebo's lifecycle. Use whichever the ROS box shows to be reliable; document the choice in a module docstring.

- [ ] **Step 2: Local static check**

Run: `cd /workspace/simulation && python3 -m py_compile src/sim_bringup/test/test_spawn_integration.py && echo compiles`
Expected: `compiles`

- [ ] **Step 3: Machine check (ROS box)**

Run:
```bash
cd /workspace/simulation && colcon build && source install/setup.bash
ros2 launch sim_bringup spawner_only.launch.py &
sleep 12
python3 -m pytest src/sim_bringup/test/test_spawn_integration.py -v
kill %1
```
Expected: `test_spawn_then_clear` PASSES — response `placed == 5`, `gz model --list` shows 5 `cart_*` before clear and 0 after.

- [ ] **Step 4: Commit**

```bash
cd /workspace/simulation
git add src/sim_bringup/test/ src/sim_bringup/package.xml
git commit -m "test: add spawn/clear integration test

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Hmj5W4Ci2Nd8JvT6M9VMcG"
```

---

## Appendix A: minimal `diffbot` fallback robot

Use only if Task 11 Step 3 shows Dolly does not load on Gazebo Harmonic. Create `models/diffbot/model.config` (like the cart's) and `models/diffbot/model.sdf`:

```xml
<?xml version="1.0" ?>
<!-- Copyright 2026 amr-sim-lab contributors. Apache-2.0. See LICENSE. -->
<sdf version="1.10">
  <model name="diffbot">
    <link name="chassis">
      <pose>0 0 0.15 0 0 0</pose>
      <inertial><mass>6.0</mass>
        <inertia><ixx>0.05</ixx><iyy>0.08</iyy><izz>0.1</izz>
        <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      <collision name="c"><geometry><box><size>0.4 0.3 0.15</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>0.4 0.3 0.15</size></box></geometry>
        <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.4 0.4 0.4 1</diffuse></material></visual>
      <sensor name="lidar" type="gpu_lidar">
        <pose>0.15 0 0.1 0 0 0</pose>
        <topic>scan</topic>
        <update_rate>10</update_rate>
        <lidar>
          <scan><horizontal><samples>360</samples><resolution>1</resolution>
            <min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle></horizontal></scan>
          <range><min>0.12</min><max>12.0</max><resolution>0.01</resolution></range>
        </lidar>
        <always_on>1</always_on><visualize>true</visualize>
      </sensor>
    </link>

    <link name="left_wheel">
      <pose>0 0.18 0.11 -1.5707 0 0</pose>
      <inertial><mass>0.5</mass><inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz>
        <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      <collision name="c"><geometry><cylinder><radius>0.11</radius><length>0.05</length></cylinder></geometry></collision>
      <visual name="v"><geometry><cylinder><radius>0.11</radius><length>0.05</length></cylinder></geometry></visual>
    </link>
    <link name="right_wheel">
      <pose>0 -0.18 0.11 -1.5707 0 0</pose>
      <inertial><mass>0.5</mass><inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz>
        <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      <collision name="c"><geometry><cylinder><radius>0.11</radius><length>0.05</length></cylinder></geometry></collision>
      <visual name="v"><geometry><cylinder><radius>0.11</radius><length>0.05</length></cylinder></geometry></visual>
    </link>
    <link name="caster">
      <pose>-0.15 0 0.05 0 0 0</pose>
      <inertial><mass>0.2</mass><inertia><ixx>0.0005</ixx><iyy>0.0005</iyy><izz>0.0005</izz>
        <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      <collision name="c"><geometry><sphere><radius>0.05</radius></sphere></geometry></collision>
      <visual name="v"><geometry><sphere><radius>0.05</radius></sphere></geometry></visual>
    </link>

    <joint name="left_wheel_joint" type="revolute">
      <parent>chassis</parent><child>left_wheel</child>
      <axis><xyz>0 0 1</xyz></axis>
    </joint>
    <joint name="right_wheel_joint" type="revolute">
      <parent>chassis</parent><child>right_wheel</child>
      <axis><xyz>0 0 1</xyz></axis>
    </joint>
    <joint name="caster_joint" type="ball">
      <parent>chassis</parent><child>caster</child>
    </joint>

    <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.36</wheel_separation>
      <wheel_radius>0.11</wheel_radius>
      <topic>cmd_vel</topic>
      <odom_topic>odometry</odom_topic>
      <frame_id>odom</frame_id>
      <child_frame_id>base_link</child_frame_id>
    </plugin>
  </model>
</sdf>
```

Then the bridge `gz_topic_name`s become `/scan` → `scan` (or `/lidar` per gz's default namespacing — confirm with `gz topic -l`), `/cmd_vel` → `/model/diffbot/cmd_vel`, `/odom` → `/model/diffbot/odometry`; set `robot` default to `diffbot` in `sim.launch.py`.

---

## Self-Review

**1. Spec coverage:**

| Spec section | Task(s) |
|---|---|
| §2 target environment / Humble+Harmonic | Task 12 (Dockerfile), Global Constraints, README (Task 13) |
| §3.1 `SpawnLayout.srv` | Task 4 |
| §3.2 `layout_generator` (pure) | Task 2 |
| §3.2 `spawner_node` (params, services, gz create/remove, startup spawn, state) | Task 5 |
| §3.3 `safety_stop` gate (pure) + node (fail-safe, front arc) | Task 6 |
| §3.4 `sim_bringup` launch + bridge + layout.yaml | Tasks 7, 8 |
| §4.1 `warehouse.sdf` | Task 9 |
| §4.2 `cart` model | Task 10 (with a documented deviation: primitives, not `.glb`) |
| §4.3 robot (Dolly, swappable) | Task 11 (+ Appendix A fallback) |
| §5 data flow | Tasks 7–8, 11 (bridge wiring) |
| §6 error handling (create timeout, saturation, invalid area, seed<0, stale/no scan, empty clear, determinism) | Task 2 tests + Task 5 (`_params_from_request`, `_on_spawn_layout`) + Task 6 (`_scan_is_fresh`) |
| §7 unit tests | Tasks 2, 3, 6 |
| §7 integration test | Task 14 |
| §7 no CI | honored (no CI task) |
| §8 Phase 2 seams | Task 2 (pure), Task 7 (canonical `layout.yaml` + root symlink), Task 10 (deviation note tracks the `.glb`), Task 13 (`gazebo-vs-isaac.md`) |
| §9 repo layout | File Structure section + all tasks |
| §10 deferred decisions | Dockerfile pins (Task 12), Dolly submodule vs copy (Task 11 Step 1), cart mesh source (Task 10 deviation), layout.yaml single-source = symlink (Task 7), README GIF = placeholder (Task 13 Step 3) |

No spec requirement is left without a task.

**2. Placeholder scan:** The only intentional "confirm on the ROS box" markers are in Tasks 5, 7, 8, 11 — each names the exact command to run and what to change. These are not deferrable ambiguities; they are hardware-in-the-loop confirmations of third-party interface names that cannot be checked in a ROS-less environment, and every one has a concrete fallback. `bridge.yaml` topic names are marked `# confirm` with the resolving command in Task 11 Step 3. No "TODO / implement later / add error handling" placeholders remain.

**3. Type consistency:**
- `LayoutParams` / `Placement` / `generate_layout(params, name_prefix)` / `InvalidArea` — defined Task 2, consumed identically Task 5.
- `include_spawn_sdf(name, x, y, yaw, *, z, model_uri)` — defined Task 3, called in Task 5 `_spawn_one` with `model_uri=uri`.
- `obstacle_in_front(ranges, angle_min, angle_increment, range_min, range_max, stop_distance, front_arc_rad)` and `apply_gate(linear_x, angular_z, blocked)` — defined Task 6 Step 3, called Task 6 Step 5 with matching argument order.
- `SpawnLayout.Request` fields (`count`, `area_min_x/y`, `area_max_x/y`, `min_spacing`, `seed`, `model_name`) / `Response` fields (`success`, `message`, `requested`, `placed`, `entity_names`) — defined Task 4, used Task 5 (`_params_from_request`, `_on_spawn_layout`) and Task 14 test.
- Node/service names: `scene_spawner` node → `~/spawn_layout`, `~/clear_layout` resolve to `/scene_spawner/spawn_layout`, `/scene_spawner/clear_layout` — used consistently in Tasks 5, 8, 13, 14.
- Parameter names identical between Task 5 (`declare_parameter`) and Task 7 (`layout.yaml`): `world_name`, `default_count`, `default_area`, `default_min_spacing`, `default_model`, `cart_footprint_radius`, `spawn_on_startup`, `startup_seed`, `create_timeout_sec`.
- `world` name `warehouse`: `<world name="warehouse">` (Task 9) == `world_name` default (Task 5) == launch arg default (Task 8).
- Topic contract: teleop → `/cmd_vel_raw` (Task 8 remap) → `safety_stop` subscribes `/cmd_vel_raw`, publishes `/cmd_vel` (Task 6) → bridge maps `/cmd_vel` ROS→GZ (Task 7). Consistent.

No inconsistencies found.

---

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-09-08-amr-sim-lab.md`. Two execution options:

**1. Subagent-Driven (recommended)** — a fresh subagent per task, review between tasks, fast iteration. Best fit here because Tasks 1–6 and 13 are fully verifiable in this environment while Tasks 5, 8–12, 14 need your ROS box — the reviewer can gate the "written + statically checked" state and hand you a punch-list of machine checks to run.

**2. Inline Execution** — execute tasks in this session with checkpoints.

Which approach?
