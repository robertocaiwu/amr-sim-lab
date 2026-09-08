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
