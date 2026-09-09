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
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("sim_bringup")
    models_path = os.path.join(pkg_share, "models")
    worlds_path = os.path.join(pkg_share, "worlds")
    world_sdf = os.path.join(worlds_path, "warehouse.sdf")

    world = LaunchConfiguration("world")

    layout_yaml = PathJoinSubstitution([FindPackageShare("sim_bringup"),
                                        "config", "layout.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="warehouse"),
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            f"{models_path}:{worlds_path}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch", "gz_sim.launch.py")),
            launch_arguments={"gz_args": [world_sdf, " -r -s --headless-rendering"]}.items(),
        ),
        Node(
            package="scene_spawner", executable="spawner_node",
            name="scene_spawner", output="screen",
            parameters=[layout_yaml, {"world_name": world}],
        ),
    ])
