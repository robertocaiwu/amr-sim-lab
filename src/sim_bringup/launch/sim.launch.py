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
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("sim_bringup")
    repo_root = os.path.abspath(os.path.join(pkg, "..", "..", "..", ".."))
    models_path = os.path.join(repo_root, "models")
    worlds_path = os.path.join(repo_root, "worlds")

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
        DeclareLaunchArgument("robot", default_value="dolly"),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH",
                               f"{models_path}:{worlds_path}"),

        Node(
            package="ros_gz_sim", executable="gz_sim", output="screen",
            arguments=[[worlds_path, "/", world, ".sdf"], "-r"],
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
                       "-x", "0", "-y", "0", "-z", "0.1"],
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


# > Note (from task brief, resolve on the ROS box): `ros_gz_sim`'s
# > launch-friendly entry is normally the included `gz_sim.launch.py`. If
# > executable="gz_sim" is not resolvable on the ROS box, replace the Gazebo
# > `Node` with an `IncludeLaunchDescription` of
# > os.path.join(get_package_share_directory("ros_gz_sim"), "launch",
# > "gz_sim.launch.py") passing `gz_args`. Confirm in Step 3 and adjust.
