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
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
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
        DeclareLaunchArgument("robot", default_value="diffbot"),
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            f"{models_path}:{worlds_path}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch", "gz_sim.launch.py")),
            launch_arguments={"gz_args": [world_sdf, " -r"]}.items(),
            condition=IfCondition(gui),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch", "gz_sim.launch.py")),
            launch_arguments={
                "gz_args": [world_sdf, " -r -s --headless-rendering"]}.items(),
            condition=UnlessCondition(gui),
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
                       "-x", "0", "-y", "0", "-z", "0.05"],
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


# > Note: resolved: Gazebo is started via the ros_gz_sim gz_sim.launch.py include.
