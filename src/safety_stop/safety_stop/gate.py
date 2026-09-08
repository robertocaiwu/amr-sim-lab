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
