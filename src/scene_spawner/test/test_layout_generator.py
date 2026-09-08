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
