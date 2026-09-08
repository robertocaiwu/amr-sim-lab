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
