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


# > Note (from task brief, resolve on the ROS box): `ros_gz_sim`'s
# > launch-friendly entry is normally the included `gz_sim.launch.py`. If
# > executable="gz_sim" is not resolvable on the ROS box, replace the Gazebo
# > `Node` with an `IncludeLaunchDescription` of
# > os.path.join(get_package_share_directory("ros_gz_sim"), "launch",
# > "gz_sim.launch.py") passing `gz_args`. Confirm in Step 3 and adjust.
