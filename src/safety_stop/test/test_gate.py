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
