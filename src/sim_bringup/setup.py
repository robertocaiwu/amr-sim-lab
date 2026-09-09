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

import os

from setuptools import find_packages, setup

package_name = 'sim_bringup'


def _recursive_data_files(dest_subdir, src_dir):
    here = os.path.dirname(os.path.abspath(__file__))
    abs_src = os.path.join(here, src_dir)
    out = []
    for root, _dirs, files in os.walk(abs_src):
        if not files:
            continue
        rel = os.path.relpath(root, abs_src)
        dest = os.path.join("share", package_name, dest_subdir) if rel == "." \
            else os.path.join("share", package_name, dest_subdir, rel)
        out.append((dest, [os.path.join(root, f) for f in files]))
    return out


data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', [
        'launch/spawner_only.launch.py', 'launch/sim.launch.py']),
    ('share/' + package_name + '/config', [
        'config/bridge.yaml', 'config/layout.yaml']),
]
data_files += _recursive_data_files("models", "../../models")
data_files += _recursive_data_files("worlds", "../../worlds")

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amr-sim-lab contributors',
    maintainer_email='you@example.com',
    description='Launch configuration and startup scripts for the simulation.',
    license='Apache-2.0',
)
