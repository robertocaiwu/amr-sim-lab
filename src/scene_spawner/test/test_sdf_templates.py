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
